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

#include "charger_backend.h"

// Charger backend for WARP chargers, controlled via the CM/UDP protocol
// implemented by the cm_networking module.
class CMChargerBackend final : public IChargerBackend {
public:
    CMChargerBackend(uint8_t charger_idx, uint8_t cm_client_id) :
        charger_idx(charger_idx), cm_client_id(cm_client_id) {}

    bool send_update(const ChargerCommand &cmd) override;
    void notify_unresponsive() override;

    // Registers the CM/UDP protocol handler for all CM-controlled chargers.
    // cm_hosts is indexed by CM client id; charger_idx_by_client_id maps a CM
    // client id to the index in the charge manager's charger arrays. Both
    // arrays must stay valid forever. Call at most once.
    static void register_all(const char *const *cm_hosts,
                             const uint8_t *charger_idx_by_client_id,
                             size_t cm_client_count);

private:
    uint8_t charger_idx;
    uint8_t cm_client_id;
};
