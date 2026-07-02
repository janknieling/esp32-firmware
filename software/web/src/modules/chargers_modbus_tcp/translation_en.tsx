/** @jsxImportSource preact */
import { h } from "preact";
let x = {
    "chargers_modbus_tcp": {
        "content": {
            "table_none": "No register table",
            "table_keba_p30": "KEBA P30 (Modbus TCP)",
            "table_custom": "Modbus/TCP (custom register table)",

            "port": "Port",
            "port_muted": "typically 502",
            "device_address": "Device address",
            "device_address_muted": "Modbus unit ID, KEBA: 255",
            "phases": "Phase connection",
            "phases_help": <><p>Number of phases the wallbox is connected with. If phase switching is used, this value only serves as the initial value; the actual state is read from the phase switching state register.</p></>,
            "phases_1": "single-phase",
            "phases_3": "three-phase",
            "phase_switch": "Phase switching",
            "phase_switch_desc": "The charge manager may switch between single and three-phase charging (requires a KEBA P30 x-series with the phase switching option)",

            "registers": "Register table",
            "registers_help": <>
                <p>Each register is assigned a role that tells the charge manager how to interpret the value. The value semantics of the roles follow the KEBA P30 Modbus documentation (e.g. charging state: 0&nbsp;start-up, 1&nbsp;not ready, 2&nbsp;ready, 3&nbsp;charging, 4&nbsp;error, 5&nbsp;suspended).</p>
                <p>Units after applying offset and scale factor: currents in mA, voltages in V, power in W, energy in Wh.</p>
                <p>At least the roles <strong>Charging state</strong> and <strong>Set charging current</strong> (or <strong>Set enabled</strong>) are required.</p>
            </>,
            "register_role": "Role",
            "register_role_help": <><p>Defines how the charge manager interprets the register value. Writable roles are written as a single holding register (function code 6).</p></>,
            "register_roles_readable": "Readable",
            "register_roles_writable": "Writable",
            "register_type": "Register type",
            "register_type_holding": "Holding register",
            "register_type_input": "Input register",
            "register_address": "Register address",
            "register_address_muted": "0-based",
            "register_value_type": "Value type",
            "register_offset": "Offset",
            "register_scale": "Scale factor",
            "register_scale_help": <><p>Value = (raw value + offset) × scale factor. Example: if the wallbox reports power in mW, a scale factor of 0.001 yields the expected unit W.</p></>,
            "register_row": /*SFN*/(role: string, addr: number) => `${role} @ ${addr}`/*NF*/,
            "register_add_title": "Add register",
            "register_add_message": /*SFN*/(have: number, max: number) => `${have} of ${max} registers configured`/*NF*/,
            "register_edit_title": "Edit register",

            "role_1": "Charging state",
            "role_2": "Cable state",
            "role_3": "Error code",
            "role_4": "Charging current L1",
            "role_5": "Charging current L2",
            "role_6": "Charging current L3",
            "role_7": "Voltage L1",
            "role_8": "Voltage L2",
            "role_9": "Voltage L3",
            "role_10": "Active power",
            "role_11": "Total energy",
            "role_12": "Charged energy (session)",
            "role_13": "Max charging current (active)",
            "role_14": "Max supported current",
            "role_15": "Phase switching state",
            "role_16": "Serial number",
            "role_17": "Firmware version",
            "role_18": "Set charging current",
            "role_19": "Set enabled",
            "role_20": "Trigger phase switch",
            "role_21": "Phase switch source",
            "role_22": "Failsafe current",
            "role_23": "Failsafe timeout",
            "role_24": "Failsafe persist",
            "role_25": "Unlock plug",
            "role_26": "Set energy limit"
        }
    }
}
