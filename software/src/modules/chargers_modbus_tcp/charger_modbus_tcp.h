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

#pragma once

#include <stdint.h>
#include <TFModbusTCPClientPool.h>

#include "config.h"
#include "modules/charge_manager/charger_backend.h"
#include "modules/modbus_tcp_client/generic_modbus_tcp_client.h"
#include "modules/modbus_tcp_client/generated/modbus_register_type.enum.h"
#include "modules/modbus_tcp_client/generated/modbus_value_type.enum.h"
#include "generated/charger_modbus_tcp_table_id.enum.h"
#include "generated/charger_modbus_tcp_register_role.enum.h"

// Largest value is 64 bit = 4 registers.
#define CHARGER_MODBUS_TCP_REGISTER_BUFFER_SIZE 4

#define CHARGER_MODBUS_TCP_ROLE_COUNT (static_cast<size_t>(ChargerModbusTCPRegisterRole::_max) + 1)

// Charger backend for third-party wallboxes controlled via Modbus/TCP with a
// table-driven register map. The builtin KEBA P30 table follows the
// "KeContact P30 Modbus TCP Programmers Guide"; custom tables assign the same
// documented role semantics to user-defined registers.
class ChargerModbusTCP final : protected GenericModbusTCPClient, public IChargerBackend
{
public:
    struct RegisterSpec {
        ChargerModbusTCPRegisterRole role;
        ModbusRegisterType register_type;
        uint16_t start_address;
        ModbusValueType value_type;
        float offset;       // value = (raw + offset) * scale_factor (reads, like meters_modbus_tcp);
        float scale_factor; // raw = value / scale_factor - offset (writes)
    };

    struct TableSpec {
        const RegisterSpec *specs;
        size_t specs_length;
        uint8_t default_device_address;
        millis_t min_read_interval;   // KEBA: reads recommended >= 0.5s apart
        millis_t min_write_interval;  // KEBA: writes recommended >= 5s apart
        millis_t keep_alive_interval; // rewrite Set Charging Current this often; 0 = disabled
        bool arm_failsafe_on_connect; // write Failsafe Current = 0 / Failsafe Timeout on connect
        uint16_t failsafe_timeout_s;
    };

    ChargerModbusTCP(uint8_t idx, const char *host, uint16_t port, TFModbusTCPClientPool *pool);

    // Called by the generator directly after construction.
    void setup_table(ChargerModbusTCPTableID table_id, const Config *table_config);
    // Called from the module's register_events(): connect once the network is up.
    void register_events();
    void pre_reboot();

    // IChargerBackend
    bool send_update(const ChargerCommand &cmd) override;
    void notify_unresponsive() override;

private:
    struct RoleValue {
        float value;       // scaled value in the role's canonical unit
        uint32_t raw;      // unscaled integer value, for state-machine roles
        bool valid;        // read at least once since (re)connect
        bool unavailable;  // charger reported an illegal-data-address error
    };

    struct WriteRequest {
        uint16_t value;
        bool pending;
    };

    void connect_callback(TFGenericTCPClientConnectResult result, TFGenericTCPClientPoolShareLevel share_level) override;
    void disconnect_callback(TFGenericTCPClientDisconnectReason reason, TFGenericTCPClientPoolShareLevel share_level) override;

    const RegisterSpec *get_spec(ChargerModbusTCPRegisterRole role) const;
    bool has_role(ChargerModbusTCPRegisterRole role) const;
    bool role_readable(ChargerModbusTCPRegisterRole role) const;

    void poll_tick();
    void read_done();
    void finish_read_cycle();

    uint8_t map_charger_state() const;
    void update_derived_state(uint8_t mapped_state);
    void publish();

    void enqueue_write(ChargerModbusTCPRegisterRole role, uint16_t value, bool only_if_changed);
    void write_tick();

    uint8_t charger_idx;
    const char *charger_host;

    const TableSpec *table = nullptr;

    // Fixed per-charger config
    uint8_t configured_phases = 3;
    bool phase_switch_enabled = false;

    // Read state
    uint8_t role_to_spec[CHARGER_MODBUS_TCP_ROLE_COUNT];
    RoleValue role_values[CHARGER_MODBUS_TCP_ROLE_COUNT] = {};
    uint16_t register_buffer[CHARGER_MODBUS_TCP_REGISTER_BUFFER_SIZE];
    size_t read_index = 0;         // index into table->specs of the read in flight
    size_t poll_cursor = 0;        // index into table->specs for the next fast poll
    size_t slow_cursor = 0;        // index into table->specs for the next slow poll
    bool statics_pending = false;  // read static roles (serial, max supported, ...) next
    bool read_in_flight = false;
    bool connected = false;
    micros_t last_static_read = 0_us;
    micros_t last_successful_value = 0_us;

    // Derived charger state
    uint32_t state_counter = 0;    // strictly increasing "uptime" for staleness detection
    uint8_t last_mapped_state = 0;
    micros_t state_change_at = 0_us;
    micros_t car_stopped_at = 0_us;
    bool box_enabled = true;       // last written Set Enabled value (default on)

    // Phase switching
    micros_t phase_switch_grace_until = 0_us;
    uint8_t phase_switch_target = 0;

    // Write state
    WriteRequest write_requests[CHARGER_MODBUS_TCP_ROLE_COUNT] = {};
    uint16_t last_written[CHARGER_MODBUS_TCP_ROLE_COUNT] = {};
    bool ever_written[CHARGER_MODBUS_TCP_ROLE_COUNT] = {};
    bool write_in_flight = false;
    micros_t last_write = 0_us;
    micros_t last_current_write = 0_us;
    uint16_t write_buffer = 0;
    uint16_t desired_current = 0;
};
