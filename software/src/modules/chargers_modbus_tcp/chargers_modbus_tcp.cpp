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

#include "chargers_modbus_tcp.h"

#include "event_log_prefix.h"
#include "generated/module_dependencies.h"

// The ChargeManager class and the charge_manager global come in via
// module_dependencies.h (Charge Manager is a required dependency).
#include "charger_modbus_tcp.h"
#include "modules/charge_manager/generated/charger_class_id.enum.h"
#include "modules/modbus_tcp_client/generated/modbus_register_type.enum.h"
#include "modules/modbus_tcp_client/generated/modbus_value_type.enum.h"
#include "generated/charger_modbus_tcp_register_role.enum.h"
#include "options.h"

void ChargersModbusTCP::pre_setup()
{
    table_custom_regs_prototype = Config::Object({
        {"role",  Config::Enum(ChargerModbusTCPRegisterRole::None)},
        {"rtype", Config::Enum(ModbusRegisterType::HoldingRegister)},
        {"addr",  Config::Uint16(0)},
        {"vtype", Config::Enum(ModbusValueType::U16)},
        {"off",   Config::Float(0.0f)},
        {"scale", Config::Float(1.0f)},
    });

    table_prototypes.push_back({ChargerModbusTCPTableID::KebaP30, Config::Object({
        {"phases",       Config::Uint(3, 1, 3)},
        {"phase_switch", Config::Bool(false)},
    })});

    table_prototypes.push_back({ChargerModbusTCPTableID::Custom, Config::Object({
        {"dev_addr",     Config::Uint8(255)},
        {"phases",       Config::Uint(3, 1, 3)},
        {"phase_switch", Config::Bool(false)},
        {"regs",         Config::Array({},
            &table_custom_regs_prototype,
            0, OPTIONS_CHARGERS_MODBUS_TCP_MAX_CUSTOM_REGISTERS(),
            Config::type_id<Config::ConfObject>())},
    })});

    ctrl_prototype = Config::Object({
        {"port", Config::Uint16(502)},
        {"table", Config::Union<ChargerModbusTCPTableID>(
            Config::Object({
                {"phases",       Config::Uint(3, 1, 3)},
                {"phase_switch", Config::Bool(false)},
            }),
            ChargerModbusTCPTableID::KebaP30,
            table_prototypes.data(),
            static_cast<uint8_t>(table_prototypes.size()))},
    });

    charge_manager.register_charger_generator(ChargerClassID::ModbusTCP, this);
}

void ChargersModbusTCP::register_events()
{
    for (ChargerModbusTCP *charger : chargers) {
        charger->register_events();
    }
}

void ChargersModbusTCP::pre_reboot()
{
    for (ChargerModbusTCP *charger : chargers) {
        charger->pre_reboot();
    }
}

const Config *ChargersModbusTCP::get_ctrl_config_prototype()
{
    return &ctrl_prototype;
}

IChargerBackend *ChargersModbusTCP::new_charger(uint8_t idx, const char *host, const Config *ctrl_config)
{
    uint16_t port = ctrl_config->get("port")->asUint16();
    auto table_id = ctrl_config->get("table")->getTag<ChargerModbusTCPTableID>();
    const Config *table_config = static_cast<const Config *>(ctrl_config->get("table")->get());

    ChargerModbusTCP *charger = new ChargerModbusTCP(idx, host, port, modbus_tcp_client.get_pool());
    charger->setup_table(table_id, table_config);

    chargers.push_back(charger);

    return charger;
}

String ChargersModbusTCP::validate_ctrl_config(const Config *ctrl_config)
{
    auto table_id = ctrl_config->get("table")->getTag<ChargerModbusTCPTableID>();

    if (table_id == ChargerModbusTCPTableID::None) {
        return "a register table must be selected for a Modbus/TCP charger";
    }

    if (table_id != ChargerModbusTCPTableID::Custom) {
        return "";
    }

    const Config *table_config = static_cast<const Config *>(ctrl_config->get("table")->get());
    const Config *regs = static_cast<const Config *>(table_config->get("regs"));

    bool have_charging_state = false;
    bool have_set_current = false;
    bool have_set_enabled = false;
    bool have_trigger_phase_switch = false;
    bool have_phase_switching_state = false;
    bool role_seen[CHARGER_MODBUS_TCP_ROLE_COUNT] = {};

    for (size_t i = 0; i < regs->count(); ++i) {
        auto role = regs->get(i)->get("role")->asEnum<ChargerModbusTCPRegisterRole>();
        uint8_t role_idx = static_cast<uint8_t>(role);

        if (role == ChargerModbusTCPRegisterRole::None) {
            return "each register of a custom table must have a role";
        }

        // Phase Switch Source may exist twice (readback + writable register).
        if (role_seen[role_idx] && role != ChargerModbusTCPRegisterRole::PhaseSwitchSource) {
            return "duplicate role in custom register table";
        }

        role_seen[role_idx] = true;

        switch (role) {
            case ChargerModbusTCPRegisterRole::ChargingState:      have_charging_state = true; break;
            case ChargerModbusTCPRegisterRole::SetChargingCurrent: have_set_current = true; break;
            case ChargerModbusTCPRegisterRole::SetEnabled:         have_set_enabled = true; break;
            case ChargerModbusTCPRegisterRole::TriggerPhaseSwitch: have_trigger_phase_switch = true; break;
            case ChargerModbusTCPRegisterRole::PhaseSwitchingState: have_phase_switching_state = true; break;
            default: break;
        }
    }

    if (!have_charging_state) {
        return "a custom register table requires a Charging State register";
    }

    if (!have_set_current && !have_set_enabled) {
        return "a custom register table requires a Set Charging Current or Set Enabled register";
    }

    if (table_config->get("phase_switch")->asBool() && (!have_trigger_phase_switch || !have_phase_switching_state)) {
        return "phase switching requires Trigger Phase Switch and Phase Switching State registers";
    }

    return "";
}
