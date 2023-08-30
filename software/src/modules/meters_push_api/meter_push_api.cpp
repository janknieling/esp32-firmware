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

#include "meter_push_api.h"
#include "module_dependencies.h"

#include "api.h"
#include "modules/meters/meter_class_defs.h"
#include "modules/meters/meter_value_id.h"

#include "gcc_warnings.h"

_ATTRIBUTE((const))
uint32_t MeterPushAPI::get_class() const
{
    return METER_CLASS_PUSH_API;
}

void MeterPushAPI::setup()
{
    IMeter::setup();

    Config *value_ids = static_cast<Config *>(config->get("value_ids"));
    ssize_t id_count = value_ids->count();
    if (id_count < 0) {
        logger.printfln("meter_push_api: Invalid ID count: %i", id_count);
        return;
    }

    value_count = static_cast<uint32_t>(id_count);
    uint16_t value_count_u16 = static_cast<uint16_t>(value_count);

    MeterValueID *ids = static_cast<MeterValueID *>(malloc(value_count * sizeof(MeterValueID)));
    for (uint16_t i = 0; i < value_count_u16; i++) {
        ids[i] = static_cast<MeterValueID>(value_ids->get(i)->asUint());
    }
    meters.declare_value_ids(slot, ids, value_count);
    free(ids);

    push_values = Config::Array({},
        meters.get_config_float_nan_prototype(),
        value_count_u16, value_count_u16, Config::type_id<Config::ConfFloat>()
    );
}

void MeterPushAPI::register_urls(String base_url)
{
    IMeter::register_urls(base_url);

    api.addCommand(base_url + "push_values", &push_values, {}, [this](){
        meters.update_all_values(this->slot, &push_values);
    }, false);
}

bool MeterPushAPI::supports_power()
{
    return support_cache_power;
}

bool MeterPushAPI::supports_energy()
{
    return support_cache_energy;
}

bool MeterPushAPI::supports_currents()
{
    return support_cache_currents;
}

/*
void MeterPushAPI::update_from_em_all_data(EnergyManagerAllData &all_data)
{
    // Reject stale data older than five seconds.
    if (deadline_elapsed(all_data.last_update + 5 * 1000))
        return;

    // Do nothing if no meter was detected.
    if (all_data.energy_meter_type == METER_TYPE_NONE)
        return;

    if (meter_type != all_data.energy_meter_type) {
        if (meter_type != METER_TYPE_NONE) {
            if (!meter_change_warning_printed) {
                logger.printfln("meter_em: Meter change detected. This is not supported.");
                meter_change_warning_printed = true;
            }
            return;
        }

        meter_type = all_data.energy_meter_type;

        MeterValueID ids[METER_ALL_VALUES_COUNT];
        uint32_t id_count = METER_ALL_VALUES_COUNT;
        sdm_helper_get_value_ids(meter_type, ids, &id_count);
        meters.declare_value_ids(slot, ids, id_count);

        value_index_power  = meters_find_id_index(ids, id_count, MeterValueID::PowerActiveLSumImExDiff);
        value_index_import = meters_find_id_index(ids, id_count, MeterValueID::EnergyActiveLSumImport);
        value_index_export = meters_find_id_index(ids, id_count, MeterValueID::EnergyActiveLSumExport);

        task_scheduler.scheduleWithFixedDelay([this](){
            update_all_values();
        }, 0, 5000);
    }

    state->get("type")->updateUint(all_data.energy_meter_type);

    errors.get("local_timeout"       )->updateUint(all_data.error_count[0]);
    errors.get("global_timeout"      )->updateUint(all_data.error_count[1]);
    errors.get("illegal_function"    )->updateUint(all_data.error_count[2]);
    errors.get("illegal_data_access" )->updateUint(all_data.error_count[3]);
    errors.get("illegal_data_value"  )->updateUint(all_data.error_count[4]);
    errors.get("slave_device_failure")->updateUint(all_data.error_count[5]);

    float power = all_data.power;
    meters.update_value(slot, value_index_power,  power);
    meters.update_value(slot, value_index_import, all_data.energy_import);
    meters.update_value(slot, value_index_export, all_data.energy_export);

    //TODO API change: replace import/export with phase currents
    //METER_ALL_VALUES_CURRENT_L1_A
    //METER_ALL_VALUES_CURRENT_L2_A
    //METER_ALL_VALUES_CURRENT_L3_A

    power_hist.add_sample(power);
}

void MeterPushAPI::update_all_values()
{
    // No need to initialize the array because either all values are written or it is rejected entirely.
    float values[METER_ALL_VALUES_COUNT];
    if (energy_manager.get_energy_meter_detailed_values(values) != METER_ALL_VALUES_COUNT)
        return;

    uint32_t values_len = ARRAY_SIZE(values);
    sdm_helper_pack_all_values(meter_type, values, &values_len);

    if (values_len == 0) {
        logger.printfln("meter_em: Cannot pack values into array of size %u.", ARRAY_SIZE(values));
    } else {
        meters.update_all_values(slot, values);
    }
}
*/