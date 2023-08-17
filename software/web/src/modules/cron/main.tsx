/* esp32-firmware
 * Copyright (C) 2023 Frederic Henrichs <frederic@tinkerforge.com>
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

import $ from "../../ts/jq";

import * as util from "../../ts/util";
import * as API from "../../ts/api";

import { Fragment, render, h, FunctionComponent, Component } from "preact";
import { ConfigComponent } from "src/ts/components/config_component";
import { SubPage } from "src/ts/components/sub_page";
import { ConfigForm } from "src/ts/components/config_form";
import { Table, TableRow } from "src/ts/components/table";
import { cron_action, cron_action_configs, cron_action_defaults, cron_action_dict, cron_trigger, cron_trigger_configs, cron_trigger_defaults, cron_trigger_dict, task } from "./api";
import { InputSelect } from "src/ts/components/input_select";
import { __ } from "src/ts/translation";

type CronConfig = API.getType['cron/config'];
type CronState = {
    edit_task: task,
    displayed_trigger: number,
    displayed_action: number
};

export class Cron extends ConfigComponent<'cron/config', {}, CronState> {
    constructor() {
        super('cron/config',
        __("charge_manager.script.save_failed"),
        __("charge_manager.script.reboot_content_changed"));
        this.state = {
            tasks: [],
            edit_task: {
                trigger: [0, null] as any as cron_trigger,
                action: [0, null] as any as cron_action
            },
            displayed_action: 0,
            displayed_trigger: 0
        }
    }

    setTriggerFromComponent(update: cron_trigger) {
        console.log("setting trigger to", update);
        let edit_task = this.state.edit_task;
        edit_task.trigger = update;
        this.setState({edit_task: edit_task});
        console.log("edit task after setting trigger", update);
    }

    setActionFromComponent(update: cron_action) {
        let edit_task = this.state.edit_task;
        edit_task.action = update;
        this.setState({edit_task: edit_task});
    }

    hackToAllowSave() {
        document.getElementById("cron-config-form").dispatchEvent(new Event('input'));
    }

    default(object: any) {
        for (let k in object) {

        }
    }

    createSelectors() {
        console.log("create selectors was called");
        let trigger: [string, string][] = [];
        for (let i in cron_trigger_configs) {
            const entry: [string, string] = [i, i]
            trigger.push(entry);
        }

        let action: [string, string][] = [];
        for (let i in cron_action_configs) {
            const entry: [string, string] = [i, i];
            action.push(entry);
        }

        let triggerSelector = [{
            name: "Trigger",
            value: <InputSelect
                        items={trigger}
                        onValue={(v) => this.setState({displayed_trigger: Number(v), edit_task: {trigger: cron_trigger_defaults[Number(v)], action: this.state.edit_task.action}})}
                        value={this.state.displayed_trigger}/>
        }]
        if (this.state.displayed_trigger != 0) {
            const trigger_config = cron_trigger_configs[this.state.displayed_trigger](this, this.state.edit_task.trigger);
            triggerSelector = triggerSelector.concat(trigger_config);
        }

        let actionSelector = [{
            name: "Action",
            value: <InputSelect
                        items={action}
                        onValue={(v) => this.setState({displayed_action: Number(v), edit_task: {action: cron_action_defaults[Number(v)], trigger: this.state.edit_task.trigger}})}
                        value={this.state.displayed_action}/>
        }]

        if (this.state.displayed_action != 0) {
            const action_config = cron_action_configs[this.state.displayed_action](this, this.state.edit_task.action);
            actionSelector = actionSelector.concat(action_config);
        }
        console.log("edit task after create selectors:", this.state.edit_task);

        return triggerSelector.concat(actionSelector);
    }

    assembleTable() {
        let rows: TableRow[] = [];
        this.state.tasks.forEach((task, idx) => {
            console.log("creating table elemen no", idx, "with action", task.action[0], "and trigger", task.trigger[0]);
            let action: number;
            let trigger: number;
            if (isNaN(Number(task.action[0]))) {
                action = task.action[0][0];
            } else {
                action = Number(task.action[0]);
            }

            if (isNaN(Number(task.trigger[0]))) {
                trigger = task.trigger[0][0];
            } else {
                trigger = Number(task.trigger[0]);
            }
            console.log("creating table elemen no", idx, "with action", action, "and trigger", trigger);
            const ActionComponent = cron_action_dict[action];
            const TriggerComponent = cron_trigger_dict[trigger];

            const task_copy = JSON.parse(JSON.stringify(task)) as task;
            let row: TableRow = {
                columnValues: [[<TriggerComponent cron={task.trigger} children={null}/>], [<ActionComponent cron={task.action} children={null}/>]],
                onEditStart: async () => {
                    this.setState({edit_task: {action: task_copy.action, trigger: task_copy.trigger}, displayed_trigger: task.trigger[0] as any as number, displayed_action: task.action[0] as any as number})
                },
                onEditGetRows: () => {
                    return this.createSelectors();
                },
                onEditCommit: async () => {
                    this.setState({tasks: this.state.tasks.map((task, k) => k === idx ? this.state.edit_task : task)});
                    this.hackToAllowSave();
                },
                onEditAbort: async () => {},
                onRemoveClick: async () => {
                    this.setState({tasks: this.state.tasks.filter((_, k) => idx == k)})
                    this.hackToAllowSave();
                }
            };
            rows.push(row);
        })
        return rows
    }

    render(props: {}, state: CronState) {
        if (!util.render_allowed())
            return <></>;

        return <SubPage>
            <ConfigForm id="cron-config-form" title="Cron config" isModified={this.isModified()} onSave={this.save} onReset={this.reset} onDirtyChange={(d) => this.ignore_updates = d}>
                <div class="mb-3">
                    <Table tableTill="md"
                        columnNames={["trigger", "action"]}
                        rows={this.assembleTable()}
                        addEnabled={true}
                        addTitle="Add Rule"
                        addMessage="Add another rule"
                        onAddStart={async () => this.setState({edit_task: {trigger: [[0, {}]], action: [[0, {}]]}})}
                        onAddGetRows={() => this.createSelectors()}
                        onAddCommit={async () => {
                            console.log("edit task at submit:", this.state.edit_task);
                            this.setState({tasks: this.state.tasks.concat([this.state.edit_task])});
                            this.hackToAllowSave();
                        }}

                        />
                </div>

            </ConfigForm>
        </SubPage>
    }
}

render(<Cron/>, $('#cron')[0]);

export function init() {
}

export function add_event_listeners(source: API.APIEventTarget) {
}

export function update_sidebar_state(module_init: any) {
    $('#sidebar-cron').prop('hidden', !module_init.cron);
}