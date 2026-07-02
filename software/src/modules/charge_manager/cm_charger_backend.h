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

#include <stddef.h>
#include <stdint.h>
#include <vector>

#include "charger_backend.h"

// Charger backend for WARP chargers, controlled via the CM/UDP protocol
// implemented by the cm_networking module.
class CMChargerBackend final : public IChargerBackend {
public:
    CMChargerBackend(uint8_t charger_idx, uint8_t cm_client_id) :
        charger_idx(charger_idx), cm_client_id(cm_client_id) {}

    bool send_update(const ChargerCommand &cmd) override;
    void notify_unresponsive() override;

private:
    uint8_t charger_idx;
    uint8_t cm_client_id;
};

class CMChargerBackendGenerator final : public IChargerBackendGenerator {
public:
    const Config *get_ctrl_config_prototype() override;
    IChargerBackend *new_charger(uint8_t idx, const char *host, const Config *ctrl_config) override;

    // Registers the CM/UDP protocol handler for all created backends. The
    // hosts and index mapping collected by new_charger() must stay valid
    // forever, so this generator must never be destroyed.
    void setup_chargers_done() override;

private:
    std::vector<const char *> cm_hosts;
    std::vector<uint8_t> charger_indices;
};
