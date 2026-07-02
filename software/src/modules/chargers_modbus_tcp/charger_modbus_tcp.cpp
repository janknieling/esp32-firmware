/* esp32-firmware
 * Copyright (C) 2026 Jan Knieling <jan.knieling@adesso.de>
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

#define EVENT_LOG_PREFIX "chargers_mbtcp"

#include "charger_modbus_tcp.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>

#include "event_log_prefix.h"
#include "generated/module_dependencies.h"

#include "modules/charge_manager/charge_manager.h"
#include "modules/charge_manager/charge_manager_private.h"
#include "tools.h"

#define MODBUS_VALUE_TYPE_TO_REGISTER_COUNT(x) (static_cast<uint8_t>(x) & 0x07)
#define MODBUS_VALUE_TYPE_TO_REGISTER_ORDER_LE(x) ((static_cast<uint8_t>(x) >> 5) & 1)

#define KEBA_PHASE_SWITCH_SOURCE_MODBUS 3

// After triggering a phase switch, report "currently switching" until the
// charger confirms the new state or this timeout elapses.
static constexpr micros_t PHASE_SWITCH_GRACE = 30_s;

// Re-read static roles (serial number, max supported current, ...) this often.
static constexpr micros_t STATIC_READ_INTERVAL = 60_min;

// If the allocator reports this charger as unresponsive and the last
// successfully read value is older than this, force a reconnect.
static constexpr micros_t FORCE_RECONNECT_THRESHOLD = 15_s;

using Role = ChargerModbusTCPRegisterRole;

// KEBA P30 c-/x-series register table, following the
// "KeContact P30 Modbus TCP Programmers Guide V1.07" and matching the KEBA
// emulation of the in-tree modbus_tcp server module. All readable values are
// UINT32 in two big-endian registers; only one value may be read per request.
static const ChargerModbusTCP::RegisterSpec keba_p30_specs[] = {
    // role,                     rtype,                              addr, vtype,                offset, scale
    {Role::ChargingState,        ModbusRegisterType::HoldingRegister, 1000, ModbusValueType::U32BE, 0.0f, 1.0f},
    {Role::CableState,           ModbusRegisterType::HoldingRegister, 1004, ModbusValueType::U32BE, 0.0f, 1.0f},
    {Role::ErrorCode,            ModbusRegisterType::HoldingRegister, 1006, ModbusValueType::U32BE, 0.0f, 1.0f},
    {Role::CurrentL1,            ModbusRegisterType::HoldingRegister, 1008, ModbusValueType::U32BE, 0.0f, 1.0f},    // mA
    {Role::CurrentL2,            ModbusRegisterType::HoldingRegister, 1010, ModbusValueType::U32BE, 0.0f, 1.0f},    // mA
    {Role::CurrentL3,            ModbusRegisterType::HoldingRegister, 1012, ModbusValueType::U32BE, 0.0f, 1.0f},    // mA
    {Role::SerialNumber,         ModbusRegisterType::HoldingRegister, 1014, ModbusValueType::U32BE, 0.0f, 1.0f},
    {Role::FirmwareVersion,      ModbusRegisterType::HoldingRegister, 1018, ModbusValueType::U32BE, 0.0f, 1.0f},
    {Role::ActivePower,          ModbusRegisterType::HoldingRegister, 1020, ModbusValueType::U32BE, 0.0f, 0.001f},  // mW -> W
    {Role::TotalEnergy,          ModbusRegisterType::HoldingRegister, 1036, ModbusValueType::U32BE, 0.0f, 0.1f},    // 0.1 Wh -> Wh
    {Role::VoltageL1,            ModbusRegisterType::HoldingRegister, 1040, ModbusValueType::U32BE, 0.0f, 1.0f},    // V
    {Role::VoltageL2,            ModbusRegisterType::HoldingRegister, 1042, ModbusValueType::U32BE, 0.0f, 1.0f},    // V
    {Role::VoltageL3,            ModbusRegisterType::HoldingRegister, 1044, ModbusValueType::U32BE, 0.0f, 1.0f},    // V
    {Role::MaxChargingCurrent,   ModbusRegisterType::HoldingRegister, 1100, ModbusValueType::U32BE, 0.0f, 1.0f},    // mA
    {Role::MaxSupportedCurrent,  ModbusRegisterType::HoldingRegister, 1110, ModbusValueType::U32BE, 0.0f, 1.0f},    // mA
    {Role::SessionEnergy,        ModbusRegisterType::HoldingRegister, 1502, ModbusValueType::U32BE, 0.0f, 0.1f},    // 0.1 Wh -> Wh
    {Role::PhaseSwitchingState,  ModbusRegisterType::HoldingRegister, 1552, ModbusValueType::U32BE, 0.0f, 1.0f},

    {Role::SetChargingCurrent,   ModbusRegisterType::HoldingRegister, 5004, ModbusValueType::U16,   0.0f, 1.0f},    // mA
    {Role::SetEnergyLimit,       ModbusRegisterType::HoldingRegister, 5010, ModbusValueType::U16,   0.0f, 10.0f},   // 10 Wh -> Wh
    {Role::SetEnabled,           ModbusRegisterType::HoldingRegister, 5014, ModbusValueType::U16,   0.0f, 1.0f},
    {Role::FailsafeCurrent,      ModbusRegisterType::HoldingRegister, 5016, ModbusValueType::U16,   0.0f, 1.0f},    // mA
    {Role::FailsafeTimeout,      ModbusRegisterType::HoldingRegister, 5018, ModbusValueType::U16,   0.0f, 1.0f},    // s
    {Role::PhaseSwitchSource,    ModbusRegisterType::HoldingRegister, 5050, ModbusValueType::U16,   0.0f, 1.0f},
    {Role::TriggerPhaseSwitch,   ModbusRegisterType::HoldingRegister, 5052, ModbusValueType::U16,   0.0f, 1.0f},
};

static const ChargerModbusTCP::TableSpec keba_p30_table = {
    keba_p30_specs,
    ARRAY_SIZE(keba_p30_specs),
    255,     // KEBA requires unit id 255
    500_ms,  // recommended read spacing
    5000_ms, // recommended write spacing
    15000_ms,
    true,    // arm failsafe on connect
    30,      // failsafe timeout, mirrors the CM protocol's 30s watchdog
};

static bool is_writable_role(Role role)
{
    return role >= Role::SetChargingCurrent;
}

static bool is_static_role(Role role)
{
    switch (role) {
        case Role::SerialNumber:
        case Role::FirmwareVersion:
        case Role::MaxSupportedCurrent:
        case Role::PhaseSwitchSource:
            return true;
        default:
            return false;
    }
}

static bool is_fast_role(Role role)
{
    switch (role) {
        case Role::ChargingState:
        case Role::CableState:
        case Role::ErrorCode:
        case Role::CurrentL1:
        case Role::CurrentL2:
        case Role::CurrentL3:
        case Role::MaxChargingCurrent:
        case Role::PhaseSwitchingState:
            return true;
        default:
            return false;
    }
}

// Order in which pending writes are sent. Failsafe first so that a fresh
// connection is safe before anything else, enable before current.
static const Role write_priority[] = {
    Role::FailsafeCurrent,
    Role::FailsafeTimeout,
    Role::FailsafePersist,
    Role::PhaseSwitchSource,
    Role::SetEnabled,
    Role::SetChargingCurrent,
    Role::TriggerPhaseSwitch,
    Role::SetEnergyLimit,
    Role::UnlockPlug,
};

static const char *format_charger_prefix(uint8_t idx, const char *host)
{
    char *buf;
    if (asprintf(&buf, "[charger %u %s] ", idx, host) < 0)
        return "[charger] ";
    return buf;
}

ChargerModbusTCP::ChargerModbusTCP(uint8_t idx, const char *host_, uint16_t port_, TFModbusTCPClientPool *pool) :
    GenericModbusTCPClient("chargers_mbtcp", format_charger_prefix(idx, host_), pool),
    charger_idx(idx),
    charger_host(host_)
{
    host = host_;
    port = port_;

    memset(role_to_spec, 0xFF, sizeof(role_to_spec));
}

void ChargerModbusTCP::setup_table(ChargerModbusTCPTableID table_id, const Config *table_config)
{
    switch (table_id) {
    case ChargerModbusTCPTableID::KebaP30:
        table = &keba_p30_table;
        configured_phases = static_cast<uint8_t>(table_config->get("phases")->asUint());
        phase_switch_enabled = table_config->get("phase_switch")->asBool();
        break;

    case ChargerModbusTCPTableID::Custom: {
            const Config *registers = static_cast<const Config *>(table_config->get("regs"));
            size_t registers_count = registers->count();

            // Leaked on purpose: charger backends are never destroyed.
            TableSpec *custom_table = new TableSpec;
            RegisterSpec *custom_specs = new RegisterSpec[registers_count];

            for (size_t i = 0; i < registers_count; ++i) {
                custom_specs[i].role = registers->get(i)->get("role")->asEnum<Role>();
                custom_specs[i].register_type = registers->get(i)->get("rtype")->asEnum<ModbusRegisterType>();
                custom_specs[i].start_address = registers->get(i)->get("addr")->asUint16();
                custom_specs[i].value_type = registers->get(i)->get("vtype")->asEnum<ModbusValueType>();
                custom_specs[i].offset = registers->get(i)->get("off")->asFloat();
                custom_specs[i].scale_factor = registers->get(i)->get("scale")->asFloat();
            }

            custom_table->specs = custom_specs;
            custom_table->specs_length = registers_count;
            custom_table->default_device_address = table_config->get("dev_addr")->asUint8();
            custom_table->min_read_interval = 500_ms;
            custom_table->min_write_interval = 5000_ms;
            custom_table->keep_alive_interval = 15000_ms;
            custom_table->failsafe_timeout_s = 30;

            table = custom_table;
            configured_phases = static_cast<uint8_t>(table_config->get("phases")->asUint());
            phase_switch_enabled = table_config->get("phase_switch")->asBool();

            custom_table->arm_failsafe_on_connect = false;
            for (size_t i = 0; i < registers_count; ++i) {
                if (custom_specs[i].role == Role::FailsafeCurrent || custom_specs[i].role == Role::FailsafeTimeout) {
                    custom_table->arm_failsafe_on_connect = true;
                }
            }
        }
        break;

    case ChargerModbusTCPTableID::None:
    default:
        logger.printfln_prefixed(event_log_prefix_override, event_log_prefix_override_len,
                                 "%sNo register table selected", event_log_message_prefix);
        return;
    }

    device_address = table->default_device_address;

    // role_to_spec maps a role to the first spec with that role that has a
    // readable value type context; used for reads and derived state.
    for (size_t i = 0; i < table->specs_length; ++i) {
        uint8_t role_idx = static_cast<uint8_t>(table->specs[i].role);

        if (role_to_spec[role_idx] == 0xFF)
            role_to_spec[role_idx] = static_cast<uint8_t>(i);
    }

    task_scheduler.scheduleWithFixedDelay([this]() {
        this->poll_tick();
        this->write_tick();
    }, 2_s, table->min_read_interval);
}

void ChargerModbusTCP::register_events()
{
    if (table == nullptr || table->specs_length == 0)
        return;

    network.on_network_connected([this](const Config *network_connected) {
        if (network_connected->asBool()) {
            start_connection();
        }
        else {
            stop_connection();
        }

        return EventResult::OK;
    });
}

void ChargerModbusTCP::pre_reboot()
{
    stop_connection();
}

const ChargerModbusTCP::RegisterSpec *ChargerModbusTCP::get_spec(Role role) const
{
    uint8_t spec_idx = role_to_spec[static_cast<uint8_t>(role)];

    if (spec_idx == 0xFF)
        return nullptr;

    return &table->specs[spec_idx];
}

bool ChargerModbusTCP::has_role(Role role) const
{
    return role_to_spec[static_cast<uint8_t>(role)] != 0xFF;
}

bool ChargerModbusTCP::role_readable(Role role) const
{
    // Writable roles are write-only on the KEBA; don't poll them. The
    // PhaseSwitchSource readback (1550) is polled via its read spec.
    return !is_writable_role(role);
}

void ChargerModbusTCP::connect_callback(TFGenericTCPClientConnectResult result, TFGenericTCPClientPoolShareLevel share_level)
{
    GenericModbusTCPClient::connect_callback(result, share_level);

    if (result != TFGenericTCPClientConnectResult::Connected)
        return;

    connected = true;

    for (size_t i = 0; i < CHARGER_MODBUS_TCP_ROLE_COUNT; ++i) {
        role_values[i].valid = false;
        role_values[i].unavailable = false;
        write_requests[i].pending = false;
        ever_written[i] = false;
    }

    generic_read_request.data[0] = register_buffer;
    generic_read_request.data[1] = nullptr;
    generic_read_request.read_twice = false;
    generic_read_request.done_callback = [this]{ this->read_done(); };

    read_in_flight = false;
    write_in_flight = false;
    poll_cursor = 0;
    slow_cursor = 0;
    statics_pending = true;

    if (table->arm_failsafe_on_connect) {
        // Charger-side watchdog: if this manager dies, the charger stops
        // charging after the failsafe timeout. The periodic Set Charging
        // Current keep-alive prevents this from firing in normal operation.
        if (has_role(Role::FailsafeCurrent))
            enqueue_write(Role::FailsafeCurrent, 0, false);
        if (has_role(Role::FailsafeTimeout))
            enqueue_write(Role::FailsafeTimeout, table->failsafe_timeout_s, false);
    }

    if (phase_switch_enabled && has_role(Role::TriggerPhaseSwitch)) {
        // Claim the phase switch for Modbus control (KEBA contact x2).
        enqueue_write(Role::PhaseSwitchSource, KEBA_PHASE_SWITCH_SOURCE_MODBUS, false);
    }
}

void ChargerModbusTCP::disconnect_callback(TFGenericTCPClientDisconnectReason reason, TFGenericTCPClientPoolShareLevel share_level)
{
    connected = false;
    read_in_flight = false;
}

void ChargerModbusTCP::poll_tick()
{
    if (!connected || read_in_flight || table == nullptr)
        return;

    bool statics_due = statics_pending || deadline_elapsed(last_static_read + STATIC_READ_INTERVAL);

    for (size_t i = poll_cursor; i < table->specs_length; ++i) {
        const RegisterSpec &spec = table->specs[i];
        Role role = spec.role;

        if (!role_readable(role))
            continue;

        if (role_values[static_cast<uint8_t>(role)].unavailable)
            continue;

        if (is_static_role(role)) {
            if (!statics_due)
                continue;
        } else if (!is_fast_role(role)) {
            if (i != slow_cursor)
                continue;
        }

        read_index = i;
        generic_read_request.register_type = spec.register_type;
        generic_read_request.start_address = spec.start_address;
        generic_read_request.register_count = MODBUS_VALUE_TYPE_TO_REGISTER_COUNT(spec.value_type);

        read_in_flight = true;
        start_generic_read();
        return;
    }

    finish_read_cycle();
}

void ChargerModbusTCP::finish_read_cycle()
{
    poll_cursor = 0;

    if (statics_pending || deadline_elapsed(last_static_read + STATIC_READ_INTERVAL)) {
        statics_pending = false;
        last_static_read = now_us();
    }

    // Advance to the next readable slow spec for the next cycle.
    for (size_t step = 0; step < table->specs_length; ++step) {
        slow_cursor = (slow_cursor + 1) % table->specs_length;

        Role role = table->specs[slow_cursor].role;

        if (role_readable(role) && !is_static_role(role) && !is_fast_role(role)
         && !role_values[static_cast<uint8_t>(role)].unavailable)
            break;
    }

    publish();
}

void ChargerModbusTCP::read_done()
{
    read_in_flight = false;

    if (generic_read_request.result != TFModbusTCPClientTransactionResult::Success) {
        if (generic_read_request.result == TFModbusTCPClientTransactionResult::ModbusIllegalDataAddress) {
            const RegisterSpec &spec = table->specs[read_index];

            // Not all registers exist on all firmware generations (e.g. the
            // firmware version register moved between KEBA P30 firmwares).
            role_values[static_cast<uint8_t>(spec.role)].unavailable = true;

            logger.printfln_prefixed(event_log_prefix_override, event_log_prefix_override_len,
                                     "%sRegister %u (role %u) not available on this charger, skipping",
                                     event_log_message_prefix, spec.start_address, static_cast<uint8_t>(spec.role));

            poll_cursor = read_index + 1;
        }

        // All other errors: retry the same register on the next tick. If the
        // connection is dead, GenericModbusTCPClient forces a reconnect after
        // one minute without a successful read.
        return;
    }

    const RegisterSpec &spec = table->specs[read_index];
    size_t register_count = MODBUS_VALUE_TYPE_TO_REGISTER_COUNT(spec.value_type);

    union {
        uint32_t u;
        float f;
        uint16_t r[2];
    } c32;

    union {
        uint64_t u;
        double f;
        uint16_t r[4];
    } c64;

    uint32_t raw = 0;
    float value = NAN;

    switch (register_count) {
    case 1:
        raw = register_buffer[0];
        value = static_cast<float>(register_buffer[0]);

        if (spec.value_type == ModbusValueType::S16)
            value = static_cast<float>(static_cast<int16_t>(register_buffer[0]));

        break;

    case 2:
        if (MODBUS_VALUE_TYPE_TO_REGISTER_ORDER_LE(spec.value_type)) {
            c32.r[0] = register_buffer[0];
            c32.r[1] = register_buffer[1];
        }
        else {
            c32.r[0] = register_buffer[1];
            c32.r[1] = register_buffer[0];
        }

        raw = c32.u;

        switch (spec.value_type) {
        case ModbusValueType::S32BE:
        case ModbusValueType::S32LE:
            value = static_cast<float>(static_cast<int32_t>(c32.u));
            break;
        case ModbusValueType::F32BE:
        case ModbusValueType::F32LE:
            value = c32.f;
            break;
        default:
            value = static_cast<float>(c32.u);
            break;
        }

        break;

    case 4:
        if (MODBUS_VALUE_TYPE_TO_REGISTER_ORDER_LE(spec.value_type)) {
            c64.r[0] = register_buffer[0];
            c64.r[1] = register_buffer[1];
            c64.r[2] = register_buffer[2];
            c64.r[3] = register_buffer[3];
        }
        else {
            c64.r[0] = register_buffer[3];
            c64.r[1] = register_buffer[2];
            c64.r[2] = register_buffer[1];
            c64.r[3] = register_buffer[0];
        }

        raw = static_cast<uint32_t>(c64.u);

        switch (spec.value_type) {
        case ModbusValueType::S64BE:
        case ModbusValueType::S64LE:
            value = static_cast<float>(static_cast<int64_t>(c64.u));
            break;
        case ModbusValueType::F64BE:
        case ModbusValueType::F64LE:
            value = static_cast<float>(c64.f);
            break;
        default:
            value = static_cast<float>(c64.u);
            break;
        }

        break;

    default:
        logger.printfln_prefixed(event_log_prefix_override, event_log_prefix_override_len,
                                 "%sValue with unsupported register count %zu", event_log_message_prefix, register_count);
        poll_cursor = read_index + 1;
        return;
    }

    value += spec.offset;
    value *= spec.scale_factor;

    RoleValue &rv = role_values[static_cast<uint8_t>(spec.role)];
    rv.value = value;
    rv.raw = raw;
    rv.valid = true;

    last_successful_value = now_us();
    poll_cursor = read_index + 1;
}

uint8_t ChargerModbusTCP::map_charger_state() const
{
    const RoleValue &cs = role_values[static_cast<uint8_t>(Role::ChargingState)];
    const RoleValue &cable = role_values[static_cast<uint8_t>(Role::CableState)];
    const RoleValue &error = role_values[static_cast<uint8_t>(Role::ErrorCode)];

    bool plugged;
    if (cable.valid) {
        plugged = cable.raw >= 5; // cable connected to the vehicle
    } else {
        // No cable state available (reduced custom table): assume a vehicle
        // is present whenever the charger is ready, charging or suspended.
        plugged = cs.raw == 2 || cs.raw == 3 || cs.raw == 5;
    }

    if (cs.raw == 4 || (error.valid && error.raw != 0))
        return 4; // error

    if (!plugged)
        return 0; // no vehicle connected

    if (cs.raw == 3)
        return 3; // charging

    if (cs.raw == 2)
        return 2; // waiting for the vehicle to start charging

    // 0 (startup), 1 (not ready: not enabled, not authorized or locked),
    // 5 (suspended, i.e. disabled by us or by the failsafe)
    return 1; // blocked
}

void ChargerModbusTCP::update_derived_state(uint8_t mapped_state)
{
    auto now = now_us();

    if (mapped_state != last_mapped_state)
        state_change_at = now;

    // The car stopped charging by itself if the charger leaves the charging
    // state towards "ready" while it is still enabled. If we disabled it, the
    // charger reports "suspended"/"not ready", which maps to blocked (1).
    if (last_mapped_state == 3 && mapped_state == 2 && box_enabled)
        car_stopped_at = now;

    if (mapped_state == 3 || mapped_state == 0)
        car_stopped_at = 0_us;

    last_mapped_state = mapped_state;
}

void ChargerModbusTCP::publish()
{
    const RoleValue &cs = role_values[static_cast<uint8_t>(Role::ChargingState)];

    // Without the charging state nothing meaningful can be reported. No
    // publish means charger_state[idx].last_update goes stale and the
    // allocator handles this charger as unreachable.
    if (!cs.valid)
        return;

    auto now = now_us();
    uint8_t mapped_state = map_charger_state();

    update_derived_state(mapped_state);

    ChargerRemoteState rs = {};

    rs.uptime = ++state_counter;
    rs.uid = role_values[static_cast<uint8_t>(Role::SerialNumber)].valid
           ? role_values[static_cast<uint8_t>(Role::SerialNumber)].raw
           : 0;
    rs.charger_state = mapped_state;
    rs.error_state = mapped_state == 4 ? 1 : 0;

    // Hardware limit of the charger. WARP semantics: 0 means "blocked by
    // another slot", so never report 0 here; blocking is signaled via
    // charger_state and allowed_current instead.
    const RoleValue &max_supported = role_values[static_cast<uint8_t>(Role::MaxSupportedCurrent)];
    uint16_t supported = 32000;
    if (max_supported.valid && max_supported.raw != 0)
        supported = static_cast<uint16_t>(std::min(max_supported.value, 32000.0f));
    rs.supported_current = std::max(supported, static_cast<uint16_t>(6000));

    // The currently active limit. Masked to 0 while the charger is disabled:
    // a disabled KEBA keeps reporting its old limit on register 1100, which
    // would otherwise trip the allocator's EVSE-nonreactive check and shut
    // down the whole site.
    const RoleValue &max_current = role_values[static_cast<uint8_t>(Role::MaxChargingCurrent)];
    uint16_t allowed = 0;
    if (cs.raw == 5 || !box_enabled) {
        allowed = 0;
    } else if (max_current.valid) {
        allowed = static_cast<uint16_t>(std::min(max_current.value, 32000.0f));
    } else if (ever_written[static_cast<uint8_t>(Role::SetChargingCurrent)]) {
        allowed = last_written[static_cast<uint8_t>(Role::SetChargingCurrent)];
    }
    rs.allowed_current = allowed;

    rs.car_stopped_charging = car_stopped_at == 0_us
                            ? 0
                            : static_cast<uint32_t>((now - car_stopped_at).to<millis_t>().as<uint64_t>());
    rs.time_since_state_change = static_cast<uint32_t>((now - state_change_at).to<millis_t>().as<uint64_t>());
    rs.time_since_state_change_known = true;

    bool has_currents = has_role(Role::CurrentL1) || has_role(Role::CurrentL2) || has_role(Role::CurrentL3);
    const Role current_roles[3] = {Role::CurrentL1, Role::CurrentL2, Role::CurrentL3};
    for (size_t i = 0; i < 3; ++i) {
        const RoleValue &rv = role_values[static_cast<uint8_t>(current_roles[i])];
        rs.line_currents[i] = rv.valid ? rv.value / 1000.0f : NAN; // mA -> A
    }

    const RoleValue &power = role_values[static_cast<uint8_t>(Role::ActivePower)];
    rs.power_total = power.valid ? power.value : NAN; // W

    const RoleValue &energy = role_values[static_cast<uint8_t>(Role::TotalEnergy)];
    rs.energy_abs = energy.valid ? energy.value / 1000.0f : NAN; // Wh -> kWh

    rs.meter_supported = has_currents || has_role(Role::ActivePower);

    // Phase information
    const RoleValue &phase_state = role_values[static_cast<uint8_t>(Role::PhaseSwitchingState)];
    bool phase_switch_usable = phase_switch_enabled
                            && has_role(Role::TriggerPhaseSwitch)
                            && phase_state.valid
                            && (phase_state.raw == 1 || phase_state.raw == 3);

    rs.phases_known = true;

    if (phase_switch_usable) {
        rs.phases = static_cast<uint8_t>(phase_state.raw);
        rs.phase_switch_supported = true;
        rs.currently_switching_phases = phase_switch_grace_until != 0_us
                                     && !deadline_elapsed(phase_switch_grace_until)
                                     && phase_state.raw != phase_switch_target;

        if (phase_state.raw == phase_switch_target)
            phase_switch_grace_until = 0_us;
    } else {
        rs.phases = configured_phases;
        rs.phase_switch_supported = false;
        rs.currently_switching_phases = false;
    }

    rs.cp_disconnect_supported = false;
    rs.cp_disconnect_state = false;

    charge_manager.ingest_remote_state(charger_idx, rs);
}

bool ChargerModbusTCP::send_update(const ChargerCommand &cmd)
{
    if (table == nullptr)
        return true; // nothing will ever happen here, don't make the send task retry

    if (!connected)
        return false;

    if (cmd.ignore_allocation) {
        // Manager just rebooted: keep the charger's current settings. The
        // failsafe covers the case that the manager never comes back.
        return true;
    }

    bool want_enabled = cmd.allocated_current > 0 && cmd.allocated_phases > 0;

    if (want_enabled) {
        uint16_t current = std::max<uint16_t>(cmd.allocated_current, 6000);
        current = std::min<uint16_t>(current, 63000);

        const RoleValue &max_supported = role_values[static_cast<uint8_t>(Role::MaxSupportedCurrent)];
        if (max_supported.valid && max_supported.raw >= 6000)
            current = std::min<uint16_t>(current, static_cast<uint16_t>(max_supported.value));

        desired_current = current;

        if (has_role(Role::SetEnabled))
            enqueue_write(Role::SetEnabled, 1, true);

        enqueue_write(Role::SetChargingCurrent, current, true);
    } else {
        // Block charging. The KEBA does not accept 0 on Set Charging Current
        // (valid range 6000-63000), so it is disabled via Set Enabled = 0
        // (suspended mode, contactor opens). Custom tables without a
        // Set Enabled role fall back to writing a current of 0, which most
        // other boxes accept as "stop charging".
        if (has_role(Role::SetEnabled)) {
            enqueue_write(Role::SetEnabled, 0, true);
        } else {
            desired_current = 0;
            enqueue_write(Role::SetChargingCurrent, 0, true);
        }
    }

    // Phase switching: only if enabled, supported and not already in progress.
    if (phase_switch_enabled && has_role(Role::TriggerPhaseSwitch)
     && (cmd.allocated_phases == 1 || cmd.allocated_phases == 3)) {
        const RoleValue &phase_state = role_values[static_cast<uint8_t>(Role::PhaseSwitchingState)];
        bool switching = phase_switch_grace_until != 0_us && !deadline_elapsed(phase_switch_grace_until);

        if (phase_state.valid && !switching && phase_state.raw != static_cast<uint32_t>(cmd.allocated_phases)) {
            enqueue_write(Role::TriggerPhaseSwitch, cmd.allocated_phases == 3 ? 1 : 0, false);
            phase_switch_target = static_cast<uint8_t>(cmd.allocated_phases);
            phase_switch_grace_until = now_us() + PHASE_SWITCH_GRACE;
        }
    }

    return true;
}

void ChargerModbusTCP::enqueue_write(Role role, uint16_t value, bool only_if_changed)
{
    uint8_t role_idx = static_cast<uint8_t>(role);

    if (only_if_changed && ever_written[role_idx] && last_written[role_idx] == value
     && !write_requests[role_idx].pending)
        return;

    write_requests[role_idx].value = value;
    write_requests[role_idx].pending = true;
}

void ChargerModbusTCP::write_tick()
{
    if (!connected || write_in_flight || table == nullptr)
        return;

    if (last_write != 0_us && !deadline_elapsed(last_write + table->min_write_interval.to<micros_t>()))
        return;

    // Keep-alive: refresh Set Charging Current periodically so that the
    // charger-side failsafe never fires while this manager is alive.
    if (table->keep_alive_interval != 0_ms
     && ever_written[static_cast<uint8_t>(Role::SetChargingCurrent)]
     && deadline_elapsed(last_current_write + table->keep_alive_interval.to<micros_t>())) {
        enqueue_write(Role::SetChargingCurrent, last_written[static_cast<uint8_t>(Role::SetChargingCurrent)], false);
    }

    for (Role role : write_priority) {
        uint8_t role_idx = static_cast<uint8_t>(role);

        if (!write_requests[role_idx].pending)
            continue;

        // Find the writable spec for this role: last spec with the role wins,
        // so that the KEBA table can have a readback spec (1550) and a
        // writable spec (5050) for the same role.
        const RegisterSpec *spec = nullptr;
        for (size_t i = 0; i < table->specs_length; ++i) {
            if (table->specs[i].role == role)
                spec = &table->specs[i];
        }

        if (spec == nullptr) {
            write_requests[role_idx].pending = false;
            continue;
        }

        uint16_t value = write_requests[role_idx].value;

        // Convert from the role's canonical unit to the register's raw value.
        if (spec->scale_factor != 1.0f || spec->offset != 0.0f) {
            float raw = static_cast<float>(value) / spec->scale_factor - spec->offset;
            value = static_cast<uint16_t>(std::max(0.0f, std::min(65535.0f, roundf(raw))));
        }

        uint16_t requested_value = write_requests[role_idx].value;

        write_buffer = value;
        write_in_flight = true;

        static_cast<TFModbusTCPSharedClient *>(connected_client)->transact(
            device_address,
            TFModbusTCPFunctionCode::WriteSingleRegister,
            spec->start_address,
            1,
            &write_buffer,
            2_s,
            [this, role, role_idx, requested_value](TFModbusTCPClientTransactionResult result, const char *error_message) {
                write_in_flight = false;
                last_write = now_us();

                if (result != TFModbusTCPClientTransactionResult::Success) {
                    logger.printfln_prefixed(event_log_prefix_override, event_log_prefix_override_len,
                                             "%sModbus error while writing role %u: %s (%d)%s%s",
                                             event_log_message_prefix,
                                             static_cast<uint8_t>(role),
                                             get_tf_modbus_tcp_client_transaction_result_name(result),
                                             static_cast<int>(result),
                                             error_message != nullptr ? " / " : "",
                                             error_message != nullptr ? error_message : "");
                    // Leave the request pending; it is retried after the
                    // write interval.
                    return;
                }

                // Only clear the request if it wasn't overwritten with a
                // newer value while this write was in flight.
                if (write_requests[role_idx].value == requested_value)
                    write_requests[role_idx].pending = false;

                last_written[role_idx] = requested_value;
                ever_written[role_idx] = true;

                if (role == Role::SetChargingCurrent)
                    last_current_write = now_us();

                if (role == Role::SetEnabled)
                    box_enabled = requested_value != 0;
            });

        return; // at most one write per tick
    }
}

void ChargerModbusTCP::notify_unresponsive()
{
    if (!connected)
        return; // reconnecting is handled by the connection backoff

    if (last_successful_value != 0_us && !deadline_elapsed(last_successful_value + FORCE_RECONNECT_THRESHOLD))
        return; // values are still coming in, probably just a slow cycle

    logger.printfln_prefixed(event_log_prefix_override, event_log_prefix_override_len,
                             "%sCharge manager reports this charger as unresponsive, reconnecting", event_log_message_prefix);

    // We might be called from a context in which disconnecting is not
    // allowed; defer like GenericModbusTCPClient does.
    task_scheduler.scheduleOnce([this]() {
        force_reconnect();
    });
}
