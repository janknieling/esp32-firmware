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

#include <vector>

#include "module.h"
#include "config.h"
#include "modules/charge_manager/charger_backend.h"
#include "generated/charger_modbus_tcp_table_id.enum.h"

class ChargerModbusTCP;

// Backend module for third-party wallboxes controlled via Modbus/TCP.
// Registers itself as a charger backend generator with the charge manager;
// chargers of this class are configured via the charge manager's chargers
// list with a builtin (KEBA P30) or a user-defined register table.
class ChargersModbusTCP final : public IModule, public IChargerBackendGenerator
{
public:
    ChargersModbusTCP() {}

    // IModule
    void pre_setup() override;
    void register_events() override;
    void pre_reboot() override;

    // IChargerBackendGenerator
    const Config *get_ctrl_config_prototype() override;
    IChargerBackend *new_charger(uint8_t idx, const char *host, const Config *ctrl_config) override;
    String validate_ctrl_config(const Config *ctrl_config) override;

private:
    Config ctrl_prototype;
    Config table_custom_regs_prototype;
    std::vector<ConfUnionPrototype<ChargerModbusTCPTableID>> table_prototypes;
    std::vector<ChargerModbusTCP *> chargers;
};
