/* esp32-firmware
 * Copyright (C) 2022 Olaf Lüke <olaf@tinkerforge.com>
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

#include "bindings/bricklet_warp_energy_manager.h"

#include "config.h"
#include "device_module.h"
#include "warp_energy_manager_bricklet_firmware_bin.embedded.h"

#define PHASE_SWITCHING_AUTOMATIC       0
#define PHASE_SWITCHING_ALWAYS_1PHASE   1
#define PHASE_SWITCHING_ALWAYS_3PHASE   2

#define RELAY_CONFIG_DEACTIVATED        0
#define RELAY_CONFIG_RULE_BASED         1
#define RELAY_CONFIG_EXTERNAL           2

#define RELAY_CONFIG_IF_INPUT3          0
#define RELAY_CONFIG_IF_INPUT4          1
#define RELAY_CONFIG_IF_PHASE_SWITCHING 2
#define RELAY_CONFIG_IF_METER           3

#define RELAY_CONFIG_IS_HIGH            0
#define RELAY_CONFIG_IS_LOW             1
#define RELAY_CONFIG_IS_1PHASE          2
#define RELAY_CONFIG_IS_2PHASE          3
#define RELAY_CONFIG_IS_GOE_0KW         4
#define RELAY_CONFIG_IS_SOE_0KW         5

#define RELAY_CONFIG_THEN_OPEN          0
#define RELAY_CONFIG_THEN_CLOSED        1

#define INPUT_CONFIG_DEACTIVATED        0
#define INPUT_CONFIG_RULES_BASED        1
#define INPUT_CONFIG_CONTACTOR_CHECK    2

#define INPUT_CONFIG_IF_HIGH            0
#define INPUT_CONFIG_IF_LOW             1

#define INPUT_CONFIG_THEN_ALLOW         0
#define INPUT_CONFIG_THEN_DISALLOW      1


typedef struct {
    bool contactor_value;

    uint8_t rgb_value_r;
    uint8_t rgb_value_g;
    uint8_t rgb_value_b;

    float power;
    float energy_relative;
    float energy_absolute;
    bool phases_active[3];
    bool phases_connected[3];

    uint8_t energy_meter_type;
    uint32_t error_count[6];

    bool input[2];
    bool output;
    uint16_t voltage;
    uint8_t contactor_check_state;
} EnergyManagerAllData;

enum SwitchingState
{
    SwitchingState_Monitoring = 0,
    SwitchingState_Stopping,
    SwitchingState_DisconnectingCP,
    SwitchingState_TogglingContactor,
    SwitchingState_ConnectingCP
};

class EnergyManager : public DeviceModule<TF_WARPEnergyManager,
                                          warp_energy_manager_bricklet_firmware_bin_data,
                                          warp_energy_manager_bricklet_firmware_bin_length,
                                          tf_warp_energy_manager_create,
                                          tf_warp_energy_manager_get_bootloader_mode,
                                          tf_warp_energy_manager_reset,
                                          tf_warp_energy_manager_destroy>
{
public:
    EnergyManager() : DeviceModule("energy_manager", "WARP Energy Manager", "Energy Manager", std::bind(&EnergyManager::setup_energy_manager, this)){}
    void pre_setup();
    void setup();
    void register_urls();
    void loop();

    // Called in energy_manager_meter setup
    void update_all_data();
    void update_all_data_struct();

    void update_io();
    void update_energy();
    void set_available_current(uint32_t current);

    void handle_relay_config_if_input(uint8_t input);
    void handle_relay_config_if_phase_switching();
    void handle_relay_config_if_meter();
    void handle_input_config_rule_based(uint8_t input);
    void handle_input_config_contactor_check(uint8_t input);
    void setup_energy_manager();
    String get_energy_manager_debug_header();
    String get_energy_manager_debug_line();

    void apply_defaults();

    bool debug = false;
    bool input_charging_allowed[2] = {true, true};

    ConfigRoot energy_manager_state;
    ConfigRoot energy_manager_config;
    ConfigRoot energy_manager_config_in_use;

    EnergyManagerAllData all_data;

private:
    SwitchingState switching_state;
    uint32_t switching_start;
    bool wants_3phase;
    bool wants_3phase_last;
    bool wants_on_last;
    bool just_switched_phases;
    uint32_t phase_state_change_blocked_until;
    uint32_t on_state_change_blocked_until;
    int32_t last_current_available_ma;
    uint32_t charge_manager_allocated_current_ma;

    // Config cache
    int32_t  max_power_from_grid_w;
    uint32_t max_current_ma;
    uint32_t min_current_ma;
    bool     excess_charging_enable;
    bool     contactor_installed;
    uint8_t  phase_switching_mode;
    uint32_t switching_hysteresis_ms;

    // Pre-calculated limits
    int32_t  overall_min_power_w;
    int32_t  threshold_3to1_w;
    int32_t  threshold_1to3_w;
};
