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

//#include "../../options.inc"

import * as options from "../../options";
import * as util from "../../ts/util";
import { h, Fragment, Component } from "preact";
import { __, translate_unchecked } from "../../ts/translation";
import { ChargerClassID } from "../charge_manager/generated/charger_class_id.enum";
import { ChargerModbusTCPTableID } from "./generated/charger_modbus_tcp_table_id.enum";
import { ChargerModbusTCPRegisterRole } from "./generated/charger_modbus_tcp_register_role.enum";
import { ModbusRegisterType } from "../modbus_tcp_client/generated/modbus_register_type.enum";
import { ModbusValueType } from "../modbus_tcp_client/generated/modbus_value_type.enum";
import { FormRow } from "../../ts/components/form_row";
import { InputNumber } from "../../ts/components/input_number";
import { InputAnyFloat } from "../../ts/components/input_any_float";
import { InputSelect } from "../../ts/components/input_select";
import { Switch } from "../../ts/components/switch";
import { Table, TableRow } from "../../ts/components/table";

export type ChargerModbusRegister = {
    role: number;
    rtype: number;
    addr: number;
    vtype: number;
    off: number;
    scale: number;
};

export type ChargerModbusTableConfig =
    [ChargerModbusTCPTableID.None, null] |
    [ChargerModbusTCPTableID.KebaP30, {
        phases: number;
        phase_switch: boolean;
    }] |
    [ChargerModbusTCPTableID.Custom, {
        dev_addr: number;
        phases: number;
        phase_switch: boolean;
        regs: ChargerModbusRegister[];
    }];

export type ChargerModbusCtrlConfig = [
    ChargerClassID.ModbusTCP,
    {
        port: number;
        table: ChargerModbusTableConfig;
    },
];

const READABLE_ROLES: number[] = [
    ChargerModbusTCPRegisterRole.ChargingState,
    ChargerModbusTCPRegisterRole.CableState,
    ChargerModbusTCPRegisterRole.ErrorCode,
    ChargerModbusTCPRegisterRole.CurrentL1,
    ChargerModbusTCPRegisterRole.CurrentL2,
    ChargerModbusTCPRegisterRole.CurrentL3,
    ChargerModbusTCPRegisterRole.VoltageL1,
    ChargerModbusTCPRegisterRole.VoltageL2,
    ChargerModbusTCPRegisterRole.VoltageL3,
    ChargerModbusTCPRegisterRole.ActivePower,
    ChargerModbusTCPRegisterRole.TotalEnergy,
    ChargerModbusTCPRegisterRole.SessionEnergy,
    ChargerModbusTCPRegisterRole.MaxChargingCurrent,
    ChargerModbusTCPRegisterRole.MaxSupportedCurrent,
    ChargerModbusTCPRegisterRole.PhaseSwitchingState,
    ChargerModbusTCPRegisterRole.SerialNumber,
    ChargerModbusTCPRegisterRole.FirmwareVersion,
];

const WRITABLE_ROLES: number[] = [
    ChargerModbusTCPRegisterRole.SetChargingCurrent,
    ChargerModbusTCPRegisterRole.SetEnabled,
    ChargerModbusTCPRegisterRole.TriggerPhaseSwitch,
    ChargerModbusTCPRegisterRole.PhaseSwitchSource,
    ChargerModbusTCPRegisterRole.FailsafeCurrent,
    ChargerModbusTCPRegisterRole.FailsafeTimeout,
    ChargerModbusTCPRegisterRole.FailsafePersist,
    ChargerModbusTCPRegisterRole.UnlockPlug,
    ChargerModbusTCPRegisterRole.SetEnergyLimit,
];

function is_writable_role(role: number) {
    return role >= ChargerModbusTCPRegisterRole.SetChargingCurrent;
}

function get_role_name(role: number) {
    return translate_unchecked(`chargers_modbus_tcp.content.role_${role}`);
}

export function new_modbus_table_config(table_id: number): ChargerModbusTableConfig {
    switch (table_id) {
        case ChargerModbusTCPTableID.KebaP30:
            return [ChargerModbusTCPTableID.KebaP30, {phases: 3, phase_switch: false}];

        case ChargerModbusTCPTableID.Custom:
            return [ChargerModbusTCPTableID.Custom, {dev_addr: 255, phases: 3, phase_switch: false, regs: []}];

        default:
            return [ChargerModbusTCPTableID.None, null];
    }
}

export function new_modbus_ctrl_config(table_id: number): ChargerModbusCtrlConfig {
    return [ChargerClassID.ModbusTCP, {
        port: 502,
        table: new_modbus_table_config(table_id),
    }];
}

export function is_modbus_ctrl(ctrl: [number, any]): ctrl is ChargerModbusCtrlConfig {
    return ctrl[0] == ChargerClassID.ModbusTCP;
}

export function get_modbus_table_id(ctrl: [number, any]): number {
    if (!is_modbus_ctrl(ctrl))
        return null;

    return ctrl[1].table[0];
}

export function get_modbus_charger_type_name(ctrl: [number, any]): string {
    switch (get_modbus_table_id(ctrl)) {
        case ChargerModbusTCPTableID.KebaP30:
            return __("chargers_modbus_tcp.content.table_keba_p30");

        case ChargerModbusTCPTableID.Custom:
            return __("chargers_modbus_tcp.content.table_custom");

        default:
            return __("chargers_modbus_tcp.content.table_none");
    }
}

interface RegisterEditorProps {
    regs: ChargerModbusRegister[];
    on_regs: (regs: ChargerModbusRegister[]) => void;
}

interface RegisterEditorState {
    register: ChargerModbusRegister;
}

class ChargerRegisterEditor extends Component<RegisterEditorProps, RegisterEditorState> {
    constructor(props: RegisterEditorProps) {
        super(props);

        this.state = {
            register: this.new_register(),
        } as RegisterEditorState;
    }

    new_register(): ChargerModbusRegister {
        return {
            role: null,
            rtype: ModbusRegisterType.HoldingRegister,
            addr: 0,
            vtype: null,
            off: 0.0,
            scale: 1.0,
        };
    }

    get_children() {
        let register = this.state.register;
        let role_known = util.hasValue(register.role);
        let writable = role_known && is_writable_role(register.role);

        return [<>
            <FormRow label={__("chargers_modbus_tcp.content.register_role")} help={__("chargers_modbus_tcp.content.register_role_help")}>
                <InputSelect
                    required
                    items={[
                        [__("chargers_modbus_tcp.content.register_roles_readable"),
                            READABLE_ROLES.map((role) => [role.toString(), get_role_name(role)]) as [string, string][]],
                        [__("chargers_modbus_tcp.content.register_roles_writable"),
                            WRITABLE_ROLES.map((role) => [role.toString(), get_role_name(role)]) as [string, string][]],
                    ]}
                    placeholder={__("select")}
                    value={role_known ? register.role.toString() : undefined}
                    onValue={(v) => {
                        let role = parseInt(v);
                        let update: Partial<ChargerModbusRegister> = {role: role};

                        // Writable registers are always written as single
                        // holding registers (function code 6).
                        if (is_writable_role(role)) {
                            update.rtype = ModbusRegisterType.HoldingRegister;
                            update.vtype = ModbusValueType.U16;
                        }

                        this.setState({register: {...register, ...update}});
                    }} />
            </FormRow>
            {!writable ?
                <FormRow label={__("chargers_modbus_tcp.content.register_type")}>
                    <InputSelect
                        required
                        items={[
                            [ModbusRegisterType.HoldingRegister.toString(), __("chargers_modbus_tcp.content.register_type_holding")],
                            [ModbusRegisterType.InputRegister.toString(), __("chargers_modbus_tcp.content.register_type_input")],
                        ]}
                        placeholder={__("select")}
                        value={util.hasValue(register.rtype) ? register.rtype.toString() : undefined}
                        onValue={(v) => {
                            this.setState({register: {...register, rtype: parseInt(v)}});
                        }} />
                </FormRow>
                : undefined}
            <FormRow label={__("chargers_modbus_tcp.content.register_address")} label_muted={__("chargers_modbus_tcp.content.register_address_muted")}>
                <InputNumber
                    required
                    min={0}
                    max={65535}
                    value={register.addr}
                    onValue={(v) => {
                        this.setState({register: {...register, addr: v}});
                    }} />
            </FormRow>
            {!writable ?
                <FormRow label={__("chargers_modbus_tcp.content.register_value_type")}>
                    <InputSelect
                        required
                        items={[
                            [ModbusValueType.U16.toString(), "U16"],
                            [ModbusValueType.S16.toString(), "S16"],
                            [ModbusValueType.U32BE.toString(), "U32BE"],
                            [ModbusValueType.U32LE.toString(), "U32LE"],
                            [ModbusValueType.S32BE.toString(), "S32BE"],
                            [ModbusValueType.S32LE.toString(), "S32LE"],
                            [ModbusValueType.F32BE.toString(), "F32BE"],
                            [ModbusValueType.F32LE.toString(), "F32LE"],
                            [ModbusValueType.U64BE.toString(), "U64BE"],
                            [ModbusValueType.U64LE.toString(), "U64LE"],
                            [ModbusValueType.S64BE.toString(), "S64BE"],
                            [ModbusValueType.S64LE.toString(), "S64LE"],
                            [ModbusValueType.F64BE.toString(), "F64BE"],
                            [ModbusValueType.F64LE.toString(), "F64LE"],
                        ]}
                        placeholder={__("select")}
                        value={util.hasValue(register.vtype) ? register.vtype.toString() : undefined}
                        onValue={(v) => {
                            this.setState({register: {...register, vtype: parseInt(v)}});
                        }} />
                </FormRow>
                : undefined}
            <FormRow label={__("chargers_modbus_tcp.content.register_offset")}>
                <InputAnyFloat
                    required
                    value={register.off}
                    onValue={(v) => {
                        this.setState({register: {...register, off: v}});
                    }} />
            </FormRow>
            <FormRow label={__("chargers_modbus_tcp.content.register_scale")} help={__("chargers_modbus_tcp.content.register_scale_help")}>
                <InputAnyFloat
                    required
                    value={register.scale}
                    onValue={(v) => {
                        this.setState({register: {...register, scale: v}});
                    }} />
            </FormRow>
        </>];
    }

    render() {
        return <Table
            nestingDepth={2}
            rows={this.props.regs.map((register, i) => {
                const row: TableRow = {
                    columnValues: [__("chargers_modbus_tcp.content.register_row")(get_role_name(register.role), register.addr)],
                    onRemoveClick: async () => {
                        this.props.on_regs(this.props.regs.filter((r, k) => k !== i));
                        return true;
                    },
                    onEditShow: async () => {
                        this.setState({register: {...register}});
                    },
                    onEditSubmit: async () => {
                        this.props.on_regs(this.props.regs.map((r, k) => k === i ? this.state.register : r));
                    },
                    onEditGetChildren: () => this.get_children(),
                    editTitle: __("chargers_modbus_tcp.content.register_edit_title"),
                };
                return row;
            })}
            columnNames={[""]}
            addEnabled={this.props.regs.length < options.CHARGERS_MODBUS_TCP_MAX_CUSTOM_REGISTERS}
            addMessage={__("chargers_modbus_tcp.content.register_add_message")(this.props.regs.length, options.CHARGERS_MODBUS_TCP_MAX_CUSTOM_REGISTERS)}
            addTitle={__("chargers_modbus_tcp.content.register_add_title")}
            onAddShow={async () => {
                this.setState({register: this.new_register()});
            }}
            onAddGetChildren={() => this.get_children()}
            onAddSubmit={async () => {
                this.props.on_regs(this.props.regs.concat([this.state.register]));
            }} />;
    }
}

interface ChargerModbusRowsProps {
    ctrl: ChargerModbusCtrlConfig;
    on_ctrl: (ctrl: ChargerModbusCtrlConfig) => void;
}

// Form rows for a Modbus/TCP charger, rendered inside the charge manager's
// add/edit charger modal below the charger type selector.
export function ChargerModbusRows(props: ChargerModbusRowsProps) {
    let table = props.ctrl[1].table;
    let table_id = table[0];

    let set_ctrl_value = (value: Partial<ChargerModbusCtrlConfig[1]>) => {
        props.on_ctrl(util.get_updated_union(props.ctrl, value) as ChargerModbusCtrlConfig);
    };

    let set_table_value = (value: any) => {
        set_ctrl_value({table: util.get_updated_union(table as any, value) as ChargerModbusTableConfig});
    };

    return <>
        <FormRow label={__("chargers_modbus_tcp.content.port")} label_muted={__("chargers_modbus_tcp.content.port_muted")}>
            <InputNumber
                required
                min={1}
                max={65535}
                value={props.ctrl[1].port}
                onValue={(v) => set_ctrl_value({port: v})} />
        </FormRow>
        {table_id == ChargerModbusTCPTableID.Custom ?
            <FormRow label={__("chargers_modbus_tcp.content.device_address")} label_muted={__("chargers_modbus_tcp.content.device_address_muted")}>
                <InputNumber
                    required
                    min={0}
                    max={255}
                    value={(table[1] as any).dev_addr}
                    onValue={(v) => set_table_value({dev_addr: v})} />
            </FormRow>
            : undefined}
        {table_id == ChargerModbusTCPTableID.KebaP30 || table_id == ChargerModbusTCPTableID.Custom ? <>
            <FormRow label={__("chargers_modbus_tcp.content.phases")} help={__("chargers_modbus_tcp.content.phases_help")}>
                <InputSelect
                    required
                    items={[
                        ["1", __("chargers_modbus_tcp.content.phases_1")],
                        ["3", __("chargers_modbus_tcp.content.phases_3")],
                    ]}
                    value={(table[1] as any).phases.toString()}
                    onValue={(v) => set_table_value({phases: parseInt(v)})} />
            </FormRow>
            <FormRow label={__("chargers_modbus_tcp.content.phase_switch")}>
                <Switch
                    desc={__("chargers_modbus_tcp.content.phase_switch_desc")}
                    checked={(table[1] as any).phase_switch}
                    onClick={() => set_table_value({phase_switch: !(table[1] as any).phase_switch})} />
            </FormRow>
        </> : undefined}
        {table_id == ChargerModbusTCPTableID.Custom ?
            <FormRow label={__("chargers_modbus_tcp.content.registers")} help={__("chargers_modbus_tcp.content.registers_help")}>
                <ChargerRegisterEditor
                    regs={(table[1] as any).regs}
                    on_regs={(regs) => set_table_value({regs: regs})} />
            </FormRow>
            : undefined}
    </>;
}
