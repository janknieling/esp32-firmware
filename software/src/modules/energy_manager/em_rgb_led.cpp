/* esp32-firmware
 * Copyright (C) 2023 Mattias Schäffersmann <mattias@tinkerforge.com>
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

#include "em_rgb_led.h"

#include "modules.h"

EmRgbLed::EmRgbLed()
{
    status = Status::OK;
    have_grid_balance = false;
    hue_balance = 0;
}

void EmRgbLed::update_led()
{
    if (status == Status::OK) {
        uint32_t H = have_grid_balance ? hue_balance : HUE_OK;
        energy_manager.set_rgb_led(TF_WARP_ENERGY_MANAGER_LED_PATTERN_BREATHING, H);
    } else {
        uint32_t H;
        if (status == Status::Warning)
            H = HUE_WARNING;
        else if (status == Status::Error)
            H = HUE_ERROR;
        else if (status == Status::Unconfigured)
            H = HUE_UNCONFIG;
        else
            H = HUE_UNKNOWN;

        energy_manager.set_rgb_led(TF_WARP_ENERGY_MANAGER_LED_PATTERN_BLINKING, H);
    }
}

void EmRgbLed::set_status(Status status)
{
    this->status = status;
    update_led();
}

void EmRgbLed::update_grid_balance(EmRgbLed::GridBalance balance)
{
    this->have_grid_balance = true;

    switch (balance) {
        case EmRgbLed::GridBalance::Export:   hue_balance = HUE_EXPORT;   break;
        case EmRgbLed::GridBalance::Balanced: hue_balance = HUE_BALANCED; break;
        case EmRgbLed::GridBalance::Import:   hue_balance = HUE_IMPORT;   break;
        default:                              hue_balance = HUE_UNKNOWN;
    }

    update_led();
}
