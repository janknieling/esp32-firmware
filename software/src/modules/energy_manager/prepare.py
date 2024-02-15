import os
import sys
import importlib.util
import importlib.machinery

software_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

def create_software_module():
    software_spec = importlib.util.spec_from_file_location('software', os.path.join(software_dir, '__init__.py'))
    software_module = importlib.util.module_from_spec(software_spec)

    software_spec.loader.exec_module(software_module)

    sys.modules['software'] = software_module

if 'software' not in sys.modules:
    create_software_module()

from software import util

import re

util.embed_bricklet_firmware_bin()

debug_log_variables = [
    "power_manager.power_at_meter_smooth_w",
    "power_manager.power_at_meter_filtered_w",
    "power_manager.power_available_w",
    "power_manager.power_available_filtered_w",
    "power_manager.charge_manager_available_current_ma",
    "power_manager.charge_manager_allocated_current_ma",
    "power_manager.max_current_limited_ma",
    "",
    "power_manager.mode",
    "power_manager.is_3phase",
    "power_manager.wants_3phase",
    "power_manager.wants_3phase_last",
    "power_manager.is_on_last",
    "power_manager.wants_on_last",
    "",
    "power_manager.charging_blocked.combined",
    "power_manager.excess_charging_enable",
    "contactor_check_tripped",
    "power_manager.just_switched_phases",
    "power_manager.uptime_past_hysteresis",
    "consecutive_bricklet_errors",
    "power_manager.switching_state",
    "",
    "power_manager.phase_state_change_blocked_until",
    "power_manager.on_state_change_blocked_until",
    "",
    "all_data.contactor_value",
    "",
    "all_data.rgb_value_r",
    "all_data.rgb_value_g",
    "all_data.rgb_value_b",
    "",
    "all_data.power",
    "all_data.current[0]",
    "all_data.current[1]",
    "all_data.current[2]",
    "",
    "all_data.energy_meter_type",
    "all_data.error_count[0]",
    "all_data.error_count[1]",
    "all_data.error_count[2]",
    "all_data.error_count[3]",
    "all_data.error_count[4]",
    "all_data.error_count[5]",
    "",
    "all_data.input[0]",
    "all_data.input[1]",
    "",
    "all_data.relay",
    "",
    "all_data.voltage",
    "",
    "all_data.contactor_check_state",
]

formats = 'fmt(' + '),\n        fmt('.join(debug_log_variables) + '),'
header  = '"' + ',"\n           "'.join([re.sub('[^.]+\.', '', v) for v in debug_log_variables]) + '"'
data    = ',\n             '.join(filter(None, debug_log_variables))

util.specialize_template("energy_manager_debug.cpp.template", "energy_manager_debug.cpp", {
    "{{{varcount}}}": str(len(debug_log_variables)),
    "{{{formats}}}": formats,
    "{{{header}}}": header,
    "{{{data}}}": data,
})
