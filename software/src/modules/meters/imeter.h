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

#pragma once

#include <stdint.h>

#include "modules/meter/value_history.h"

class IMeter
{
public:
    virtual ~IMeter() = default;

    virtual uint32_t get_class() const = 0;
    virtual void setup() {power_hist.setup();}
    virtual void register_urls(String base_url) {power_hist.register_urls(base_url);}

    virtual bool supports_power() {return false;}
    virtual bool get_power(float *power_w) {return false;}

    virtual bool supports_import_export() {return false;}
    virtual bool get_import_export(float *energy_import_kwh, float *energy_export_kwh) {return false;}

    virtual bool supports_line_currents() {return false;}
    virtual bool get_line_currents(float *l1_current_ma, float *l2_current_ma, float *l3_current_ma) {return false;}

    //virtual bool supports_phases() {return false;}
    //virtual bool get_phases(/* TODO */) {return false;}

protected:
    ValueHistory power_hist;
};