/* esp32-firmware
 * Copyright (C) 2020-2021 Erik Fleckstein <erik@tinkerforge.com>
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

#include <functional>

#include "charge_manager_private.h"
#include "charger_backend.h"

int allocate_current(
        const CurrentAllocatorConfig *cfg,
        CurrentLimits *limits,
        const bool cp_disconnect_requested,
        /*const TODO: move allocated_energy into charger allocation state so that charger_state can be const once again*/ ChargerState *charger_state,
        const char * const *hosts,
        const std::function<const char *(uint8_t)> &get_charger_name,
        const std::function<void (uint8_t)> &notify_charger_unresponsive,

        CurrentAllocatorState *ca_state,
        ChargerAllocationState *charger_allocation_state,
        ChargerDecision *charger_decisions);

void update_charger_state(
    uint8_t idx,
    const ChargerRemoteState &rs,
    const CurrentAllocatorConfig *cfg,
    ChargerState *charger_state,
    ChargerAllocationState *charger_allocation_state);

void apply_cost(const Cost &cost, CurrentLimits* limits);

Cost get_cost(int current_to_allocate,
              uint8_t phases_to_allocate,
              PhaseRotation rot,
              int allocated_current,
              uint8_t allocated_phases);
