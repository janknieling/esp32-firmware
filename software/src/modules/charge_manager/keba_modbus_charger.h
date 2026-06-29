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

#pragma once

#include "generated/module_available.h"

#if MODULE_MODBUS_TCP_CLIENT_AVAILABLE()

#include <stdint.h>

#include "modules/modbus_tcp_client/generic_modbus_tcp_client.h"

struct ChargerState;
struct ChargerAllocationState;

// Backend that lets the charge manager control a third-party KEBA KeContact P30
// (x-series) wallbox via Modbus/TCP, so it participates in load management
// together with the WARP chargers. The register map follows the
// "KeContact P30 Modbus TCP Programmers Guide" V1.07.
//
// One instance is created per charge-manager charger slot whose protocol is set
// to KEBA Modbus/TCP. It polls the wallbox state into the slot's ChargerState
// and writes the allocation decision (current, enable, phase switch) back to the
// wallbox. A Modbus failsafe is configured on connect so the wallbox stops
// charging if the energy manager loses communication.
class KebaModbusCharger final : public GenericModbusTCPClient
{
public:
    // Indices into the polled-register tables. Keep in sync with keba_poll_address[].
    enum PollReg : uint8_t {
        POLL_STATE = 0,    // 1000 charging state
        POLL_CABLE,        // 1004 cable state
        POLL_CURRENT_L1,   // 1008 current L1 [mA]
        POLL_CURRENT_L2,   // 1010 current L2 [mA]
        POLL_CURRENT_L3,   // 1012 current L3 [mA]
        POLL_ACTIVE_POWER, // 1020 active power [mW]
        POLL_TOTAL_ENERGY, // 1036 total energy [0.1 Wh]
        POLL_VOLTAGE_L1,   // 1040 voltage L1 [V]
        POLL_VOLTAGE_L2,   // 1042 voltage L2 [V]
        POLL_VOLTAGE_L3,   // 1044 voltage L3 [V]
        POLL_MAX_CURRENT,  // 1100 max charging current [mA]
        POLL_MAX_SUPPORTED,// 1110 max supported current [mA]
        POLL_SESSION_ENERGY,//1502 charged energy this session [0.1 Wh]
        POLL_PHASE_STATE,  // 1552 phase switching state (1 or 3)
        POLL_SERIAL,       // 1014 serial number
        POLL_PRODUCT,      // 1016 product type and features
        POLL_COUNT
    };

    KebaModbusCharger(uint8_t slot, ChargerState *charger_state, ChargerAllocationState *charger_allocation_state,
                      const String &host_, uint16_t port_, bool enable_phase_switch_, TFModbusTCPClientPool *pool);

    // Start connecting and polling. Safe to call once during setup.
    void begin();

    // Record the latest allocation decision. The values are written to the
    // wallbox asynchronously by the poll loop (single in-flight transaction).
    void apply_allocation(uint16_t allocated_current, int8_t allocated_phases);

private:
    void connect_callback(TFGenericTCPClientConnectResult result, TFGenericTCPClientPoolShareLevel share_level) override;
    void disconnect_callback(TFGenericTCPClientDisconnectReason reason, TFGenericTCPClientPoolShareLevel share_level) override;

    void start_cycle();
    void schedule_next_cycle();

    void build_write_queue();
    void run_writes();
    void write_register(uint16_t address, uint16_t value, std::function<void(bool)> &&done);

    void start_reads();
    void read_one();
    void read_one_done();

    void apply_state();

    uint8_t slot;
    ChargerState *cs;
    ChargerAllocationState *cas;
    bool enable_phase_switch;

    bool connected = false;

    // Polling
    uint64_t cycle_task_id = 0;
    size_t poll_index = 0;
    uint16_t read_buf[2] = {0, 0};
    uint32_t reg_value[POLL_COUNT] = {0};
    bool reg_valid[POLL_COUNT] = {false};
    micros_t charging_since = 0_us;
    uint8_t last_iec_state = 0;

    // Desired allocation (set by apply_allocation, consumed by the write queue)
    bool have_allocation = false;
    uint16_t desired_current = 0;
    int8_t desired_phases = 0;

    // One-time per-connection configuration
    bool failsafe_configured = false;
    bool phase_source_configured = false;

    // Write throttling / keepalive bookkeeping
    int last_enable_written = -1;     // -1 unknown, 0 disabled, 1 enabled
    uint16_t last_current_written = 0;
    int last_phase_written = -1;      // -1 unknown, else 1 or 3
    micros_t last_keepalive = 0_us;

    // Pending write queue for the current cycle
    struct WriteOp {
        uint16_t address;
        uint16_t value;
    };
    WriteOp write_queue[8];
    size_t write_count = 0;
    size_t write_index = 0;
    uint16_t write_value_buf = 0; // backing storage for the in-flight FC6 write
};

#endif

#include "generated/module_available_end.h"
