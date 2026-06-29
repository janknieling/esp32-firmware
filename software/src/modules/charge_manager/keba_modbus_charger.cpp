/* esp32-firmware
 * Copyright (C) 2025 Jan Knieling
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#define EVENT_LOG_PREFIX "keba_mbtcp"

#include "keba_modbus_charger.h"

#if MODULE_MODBUS_TCP_CLIENT_AVAILABLE()

#include <algorithm>
#include <math.h>

#include "event_log_prefix.h"
#include "generated/module_dependencies.h"
#include "tools.h"
#include "charge_manager_private.h"

#include "gcc_warnings.h"

// Polled holding-register addresses (FC3). Order must match KebaModbusCharger::PollReg.
// The KEBA returns UINT32 (2 words) per register and does not allow batch reads,
// so every register is read individually with a count of 2 words.
static const uint16_t keba_poll_address[KebaModbusCharger::POLL_COUNT] = {
    1000, // POLL_STATE
    1004, // POLL_CABLE
    1008, // POLL_CURRENT_L1
    1010, // POLL_CURRENT_L2
    1012, // POLL_CURRENT_L3
    1020, // POLL_ACTIVE_POWER
    1036, // POLL_TOTAL_ENERGY
    1040, // POLL_VOLTAGE_L1
    1042, // POLL_VOLTAGE_L2
    1044, // POLL_VOLTAGE_L3
    1100, // POLL_MAX_CURRENT
    1110, // POLL_MAX_SUPPORTED
    1502, // POLL_SESSION_ENERGY
    1552, // POLL_PHASE_STATE
    1014, // POLL_SERIAL
    1016, // POLL_PRODUCT
};

// Writeable register addresses (FC6, single UINT16 register).
static constexpr uint16_t KEBA_REG_SET_CURRENT       = 5004; // [mA] 6000..63000
static constexpr uint16_t KEBA_REG_ENABLE            = 5014; // 0 disable, 1 enable
static constexpr uint16_t KEBA_REG_PHASE_SWITCH_SRC  = 5050; // 3 = toggle via Modbus
static constexpr uint16_t KEBA_REG_PHASE_SWITCH_TRIG = 5052; // 0 = 1 phase, 1 = 3 phases
static constexpr uint16_t KEBA_REG_FAILSAFE_CURRENT  = 5016; // [mA] 0 = stop charging
static constexpr uint16_t KEBA_REG_FAILSAFE_TIMEOUT  = 5018; // [s] 5..600, 0 = off

static constexpr uint8_t KEBA_UNIT_ID = 255; // mandated by the KEBA Modbus TCP guide

static constexpr uint16_t KEBA_MIN_CURRENT = 6000;  // [mA] minimum the KEBA accepts on 5004
static constexpr uint16_t KEBA_MAX_CURRENT = 32000; // [mA] clamp to the charge manager's range

// Stop charging if the energy manager stops talking to the KEBA for this long.
static constexpr uint16_t KEBA_FAILSAFE_TIMEOUT_S = 60;
// Re-assert current/enable at least this often so the failsafe timeout never trips during normal operation.
static constexpr micros_t KEBA_KEEPALIVE_INTERVAL = 25_s;
// Interval between poll cycles. The KEBA guide recommends >0.5 s between reads.
static constexpr micros_t KEBA_POLL_INTERVAL = 1_s;

// Mirror of charge_manager's defaults; used for the measured-current based requested_current.
static constexpr int KEBA_REQUESTED_CURRENT_MARGIN = 3000; // [mA]
static constexpr micros_t KEBA_REQUESTED_CURRENT_THRESHOLD = 60_s;

KebaModbusCharger::KebaModbusCharger(uint8_t slot_, ChargerState *charger_state, ChargerAllocationState *charger_allocation_state,
                                     const String &host_, uint16_t port_, bool enable_phase_switch_, TFModbusTCPClientPool *pool) :
    GenericModbusTCPClient("keba_mbtcp", "", pool),
    slot(slot_),
    cs(charger_state),
    cas(charger_allocation_state),
    enable_phase_switch(enable_phase_switch_)
{
    host = host_;
    port = port_;
    device_address = KEBA_UNIT_ID;
}

void KebaModbusCharger::begin()
{
    start_connection();

    // The poll loop reschedules itself; start it once. It is a no-op while disconnected.
    cycle_task_id = task_scheduler.scheduleOnce([this]() {
        this->start_cycle();
    }, KEBA_POLL_INTERVAL);
}

void KebaModbusCharger::connect_callback(TFGenericTCPClientConnectResult result, TFGenericTCPClientPoolShareLevel share_level)
{
    GenericModbusTCPClient::connect_callback(result, share_level);

    if (result != TFGenericTCPClientConnectResult::Connected) {
        return;
    }

    logger.printfln("KEBA charger %u connected (%s:%u)", slot, host.c_str(), port);

    connected = true;

    // (Re-)configure the one-time settings and re-assert the allocation after a (re)connect.
    failsafe_configured = false;
    phase_source_configured = false;
    last_enable_written = -1;
    last_current_written = 0;
    last_phase_written = -1;
    last_keepalive = 0_us;
}

void KebaModbusCharger::disconnect_callback(TFGenericTCPClientDisconnectReason reason, TFGenericTCPClientPoolShareLevel share_level)
{
    connected = false;

    // Stop refreshing last_update: after CHARGER_UNREACHABLE_TIMEOUT the charge
    // manager treats the slot as unreachable and stops allocating to it. The
    // KEBA's own Modbus failsafe is the hardware backstop.
}

void KebaModbusCharger::apply_allocation(uint16_t allocated_current, int8_t allocated_phases)
{
    have_allocation = true;
    desired_current = allocated_current;
    desired_phases = allocated_phases;
}

void KebaModbusCharger::schedule_next_cycle()
{
    cycle_task_id = task_scheduler.scheduleOnce([this]() {
        this->start_cycle();
    }, KEBA_POLL_INTERVAL);
}

void KebaModbusCharger::start_cycle()
{
    if (!connected || connected_client == nullptr) {
        schedule_next_cycle();
        return;
    }

    build_write_queue();
    write_index = 0;
    run_writes();
}

void KebaModbusCharger::build_write_queue()
{
    write_count = 0;

    auto push = [this](uint16_t address, uint16_t value) {
        if (write_count < ARRAY_SIZE(write_queue)) {
            write_queue[write_count].address = address;
            write_queue[write_count].value = value;
            ++write_count;
        }
    };

    // 1) One-time per-connection failsafe so the KEBA stops if we lose comms.
    if (!failsafe_configured) {
        push(KEBA_REG_FAILSAFE_CURRENT, 0); // 0 = deactivate charging on failsafe
        push(KEBA_REG_FAILSAFE_TIMEOUT, KEBA_FAILSAFE_TIMEOUT_S);
        failsafe_configured = true;
    }

    // 2) One-time per-connection phase switch source = Modbus.
    if (enable_phase_switch && !phase_source_configured) {
        push(KEBA_REG_PHASE_SWITCH_SRC, 3); // 3 = toggle via Modbus
        phase_source_configured = true;
    }

    if (!have_allocation) {
        return;
    }

    bool enable = desired_phases > 0 && desired_current >= KEBA_MIN_CURRENT;
    bool keepalive_due = deadline_elapsed(last_keepalive + KEBA_KEEPALIVE_INTERVAL);

    if (!enable) {
        // Disable charging. Writing 5014 = 0 stops an active session.
        if (last_enable_written != 0 || keepalive_due) {
            push(KEBA_REG_ENABLE, 0);
            last_enable_written = 0;
            last_keepalive = now_us();
        }
    } else {
        uint16_t current = std::min<uint16_t>(std::max<uint16_t>(desired_current, KEBA_MIN_CURRENT), KEBA_MAX_CURRENT);

        if (last_current_written != current || keepalive_due) {
            push(KEBA_REG_SET_CURRENT, current);
            last_current_written = current;
        }

        if (last_enable_written != 1 || keepalive_due) {
            push(KEBA_REG_ENABLE, 1);
            last_enable_written = 1;
        }

        if (keepalive_due) {
            last_keepalive = now_us();
        }

        // Phase switch: 5052 with 0 (1 phase) or 1 (3 phases). Only when the
        // wallbox actually reports a different phase state, to avoid needless toggles.
        if (enable_phase_switch && phase_source_configured) {
            int target_phases = desired_phases >= 3 ? 3 : 1;
            int reported_phases = reg_valid[POLL_PHASE_STATE] ? (int)reg_value[POLL_PHASE_STATE] : target_phases;

            if (target_phases != reported_phases && last_phase_written != target_phases) {
                push(KEBA_REG_PHASE_SWITCH_TRIG, target_phases >= 3 ? 1 : 0);
                last_phase_written = target_phases;
            } else if (target_phases == reported_phases) {
                last_phase_written = target_phases;
            }
        }
    }
}

void KebaModbusCharger::run_writes()
{
    if (write_index >= write_count) {
        start_reads();
        return;
    }

    const WriteOp &op = write_queue[write_index];

    write_register(op.address, op.value, [this](bool /*ok*/) {
        ++write_index;
        run_writes();
    });
}

void KebaModbusCharger::write_register(uint16_t address, uint16_t value, std::function<void(bool)> &&done)
{
    if (connected_client == nullptr) {
        done(false);
        return;
    }

    write_value_buf = value;

    static_cast<TFModbusTCPSharedClient *>(connected_client)->transact(
        device_address,
        TFModbusTCPFunctionCode::WriteSingleRegister,
        address,
        1,
        &write_value_buf,
        2_s,
        [this, address, done](TFModbusTCPClientTransactionResult result, const char *error_message) {
            bool ok = result == TFModbusTCPClientTransactionResult::Success;

            if (!ok) {
                logger.printfln("KEBA charger %u: write to register %u failed: %s (%d)%s%s",
                                slot, address,
                                get_tf_modbus_tcp_client_transaction_result_name(result),
                                static_cast<int>(result),
                                error_message != nullptr ? " / " : "",
                                error_message != nullptr ? error_message : "");
            }

            done(ok);
        });
}

void KebaModbusCharger::start_reads()
{
    // Invalidate previous values so apply_state() never uses stale data from an
    // earlier cycle for a register that failed to read this cycle.
    for (size_t i = 0; i < POLL_COUNT; ++i) {
        reg_valid[i] = false;
    }

    poll_index = 0;
    read_one();
}

void KebaModbusCharger::read_one()
{
    if (poll_index >= POLL_COUNT) {
        apply_state();
        schedule_next_cycle();
        return;
    }

    generic_read_request.register_type = ModbusRegisterType::HoldingRegister;
    generic_read_request.start_address = keba_poll_address[poll_index];
    generic_read_request.register_count = 2; // KEBA registers are UINT32 (2 words)
    generic_read_request.data[0] = read_buf;
    generic_read_request.data[1] = nullptr;
    generic_read_request.read_twice = false;
    generic_read_request.done_callback = [this]() {
        this->read_one_done();
    };

    start_generic_read();
}

void KebaModbusCharger::read_one_done()
{
    if (generic_read_request.result == TFModbusTCPClientTransactionResult::Success) {
        reg_value[poll_index] = (static_cast<uint32_t>(read_buf[0]) << 16) | read_buf[1];
        reg_valid[poll_index] = true;
        ++poll_index;
        read_one();
        return;
    }

    // Connection lost: abort the rest of this cycle and try again next time.
    if (generic_read_request.result == TFModbusTCPClientTransactionResult::NotConnected
     || generic_read_request.result == TFModbusTCPClientTransactionResult::Aborted) {
        schedule_next_cycle();
        return;
    }

    // Transient error (e.g. timeout) on a single register: mark invalid and continue.
    reg_valid[poll_index] = false;
    ++poll_index;
    read_one();
}

void KebaModbusCharger::apply_state()
{
    // Require the two registers that drive the state machine, otherwise don't
    // refresh last_update so the charge manager's watchdog can react.
    if (!reg_valid[POLL_STATE] || !reg_valid[POLL_CABLE]) {
        return;
    }

    const micros_t now = now_us();

    const uint32_t keba_state = reg_value[POLL_STATE]; // 0 startup,1 not ready,2 ready,3 charging,4 error,5 suspended
    const uint32_t cable_state = reg_value[POLL_CABLE]; // 5/7 => cable connected to the vehicle
    const bool vehicle_present = cable_state == 5 || cable_state == 7;
    const bool error = keba_state == 4;

    // Map KEBA state to the charge manager's IEC-61851-like state (0=A,1=B blocked,2=B waiting,3=C charging,4=error).
    uint8_t iec_state;
    if (error) {
        iec_state = 4;
    } else if (!vehicle_present) {
        iec_state = 0;
    } else if (keba_state == 3) {
        iec_state = 3;
    } else {
        iec_state = 2; // vehicle connected, ready/suspended/waiting
    }

    // supported/allowed currents
    uint16_t supported_current = 0;
    if (reg_valid[POLL_MAX_SUPPORTED]) {
        supported_current = static_cast<uint16_t>(std::min<uint32_t>(reg_value[POLL_MAX_SUPPORTED], KEBA_MAX_CURRENT));
    }
    uint16_t allowed_current = 0;
    if (reg_valid[POLL_MAX_CURRENT]) {
        allowed_current = static_cast<uint16_t>(std::min<uint32_t>(reg_value[POLL_MAX_CURRENT], KEBA_MAX_CURRENT));
    }

    // No per-user limiting for a third-party box: always allow charging.
    cs->user_current = KEBA_MAX_CURRENT;

    cs->wants_to_charge = ((supported_current != 0 && (iec_state == 1 || iec_state == 2)) || iec_state == 3) && cs->user_current > 0;
    cs->wants_to_charge_low_priority = false;
    cs->is_charging = iec_state == 3 && cs->user_current > 0;

    if (iec_state != 1 && iec_state != 2) {
        cs->last_wakeup = 0_us;
    }

    if (iec_state == 0) {
        cs->allocated_energy = 0;
        cs->allocated_average_power = 0;
        cas->allocated_current = 0;
        cas->allocated_phases = 0;
    }

    cs->allowed_current = allowed_current;
    cs->supported_current = supported_current;
    cs->cp_disconnect_supported = false;
    cs->cp_disconnect_state = false;

    // Plug-in / plug-out transitions, mirroring update_from_client_packet().
    if (cs->charger_state == 0 && iec_state != 0) {
        cs->last_plug_in = now;
        if (cs->last_update != 0_us)
            cs->just_plugged_in_timestamp = now;
    }
    if (iec_state == 0 && (cs->charger_state != 0 || cs->last_update == 0_us)) {
        cs->last_plug_out = now;
    }
    if (cs->charger_state != iec_state && iec_state == 3) {
        cs->last_phase_switch = now;
        charging_since = now;
    }
    if (cs->charger_state == 3 && iec_state == 2) {
        cs->last_phase_switch = now;
    }
    if (iec_state == 0) {
        cs->last_phase_switch = 0_us; // global_hysteresis is applied by the allocator on (re)start
        cs->time_in_state_c = 0_us;
        cs->last_plug_in = 0_us;
        cs->last_switch_on = 0_us;
        charging_since = 0_us;
    }

    cs->charger_state = iec_state;
    cs->last_update = now;
    last_iec_state = iec_state;

    // requested_current: ramp up fast using supported_current, then fall back to the
    // measured peak phase current + margin once the car has been charging for a while.
    uint16_t requested_current = supported_current;
    if (iec_state == 3 && charging_since != 0_us
     && (now - charging_since) >= KEBA_REQUESTED_CURRENT_THRESHOLD) {
        int max_phase_current = -1;
        const PollReg current_regs[3] = {POLL_CURRENT_L1, POLL_CURRENT_L2, POLL_CURRENT_L3};
        bool all_valid = true;
        for (int i = 0; i < 3; ++i) {
            if (!reg_valid[current_regs[i]]) {
                all_valid = false;
                break;
            }
            max_phase_current = std::max<int>(max_phase_current, (int)reg_value[current_regs[i]]); // already in mA
        }
        if (!all_valid || max_phase_current == 0) {
            max_phase_current = KEBA_MAX_CURRENT;
        }
        max_phase_current += KEBA_REQUESTED_CURRENT_MARGIN;
        max_phase_current = std::max(6000, std::min((int)KEBA_MAX_CURRENT, max_phase_current));
        requested_current = std::min<uint16_t>(supported_current, (uint16_t)max_phase_current);
    }
    cs->requested_current = requested_current;

    // Phases and metering.
    if (reg_valid[POLL_PHASE_STATE]) {
        uint8_t phases = reg_value[POLL_PHASE_STATE] == 3 ? 3 : 1;
        cs->phases = phases;
    } else if (cs->phases == 0) {
        cs->phases = enable_phase_switch ? 1 : 3;
    }
    cs->phase_switch_supported = enable_phase_switch;
    cs->currently_switching_phases = false;

    cs->meter_supported = true;
    if (reg_valid[POLL_ACTIVE_POWER]) {
        cs->power_total_sum = cs->power_total_sum + (float)reg_value[POLL_ACTIVE_POWER] / 1000.0f; // mW -> W
        cs->power_total_count = cs->power_total_count + 1;
    }
    if (reg_valid[POLL_TOTAL_ENERGY]) {
        cs->energy_abs = (float)reg_value[POLL_TOTAL_ENERGY] * 0.0001f; // 0.1 Wh -> kWh
    }

    // UI state and error, mirroring the WARP receive path.
    if (error) {
        cas->state = CASState::Error;
    } else {
        cas->error = CASError::OK;
        if (iec_state == 0) {
            cas->state = CASState::NoVehicle;
        } else if (iec_state == 3) {
            cas->state = CASState::Charging;
        } else if (cs->is_charging) {
            cas->state = CASState::Charging;
        } else if (cas->allocated_current != 0) {
            cas->state = CASState::CarBlocked; // current allocated, waiting for the car
        } else {
            cas->state = CASState::ManagerBlocked; // no current allocated yet
        }
    }
}

#endif
