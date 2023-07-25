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

import $ from "../../ts/jq";

import * as util from "../../ts/util";
import * as API from "../../ts/api";


import { h, render, Fragment, Component } from "preact";
import { __ } from "../../ts/translation";

import { InputText } from "../../ts/components/input_text";
import { SubPage } from "src/ts/components/sub_page";
import { PageHeader } from "src/ts/components/page_header";
import { Table } from "src/ts/components/table";
import { InputSelect } from "src/ts/components/input_select";

interface State {
    editCert: API.getType['certs/add'] & {'file': File}
    addCert: API.getType['certs/add'] & {'file': File}
}

const MAX_CERTS = 8;


export class Certs extends Component<{}, State> {
    render(props: {}, state: Readonly<State>) {
        if (!util.render_allowed())
            return <></>

        return (
            <SubPage>
                <PageHeader title={__("certs.content.certs")}/>
                    <div class="mb-3">
                        <Table
                            tableTill="md"
                            columnNames={["name", "size"]}
                            rows={API.get('certs/state').certs.map((cert, i) =>
                                { return {
                                    columnValues: [
                                        [cert.name],
                                        [cert.size]
                                    ],
                                    editTitle: __("nfc.content.edit_tag_title"),
                                    onEditStart: async () => this.setState({editCert: {id: cert.id, name: cert.name, cert: "", file: null}}),
                                    onEditGetRows: () => [
                                        {
                                            name: __("nfc.content.edit_tag_tag_id"),
                                            value: <InputText value={state.editCert.name}
                                                            onValue={(v) => this.setState({editCert: {...state.editCert, name: v}})}
                                                            maxLength={32}
                                                            invalidFeedback={__("nfc.content.tag_id_invalid_feedback")}
                                                            required/>
                                        },
                                        {
                                            name: "Select cert",
                                            value: <div class="custom-file">
                                                        <input type="file" class="custom-file-input form-control" accept={"application/pem-certificate-chain"}
                                                            onChange={(ev) => this.setState({editCert: {...state.editCert, file: (ev.target as HTMLInputElement).files[0]}})}/>
                                                        <label class="custom-file-label form-control rounded-left"
                                                            data-browse={"browse"}>{state.editCert.file ? state.editCert.file.name : "select file"}</label>
                                                    </div>
                                        }
                                    ],
                                    onEditCommit: async () => {
                                        // TODO: size check
                                        await API.call('certs/modify', {
                                            id: state.editCert.id,
                                            name: state.editCert.name,
                                            cert: await state.editCert.file.text()
                                        }, "error_string");

                                        this.setState({
                                            editCert: {id: 0, name: "", cert: "", file: null}
                                        });
                                    },
                                    onEditAbort: async () => this.setState({editCert: {id: 0, name: "", cert: "", file: null}}),
                                    onRemoveClick: async () => API.call('certs/remove', {id: cert.id}, "error_string")
                                }})
                            }
                            addEnabled={API.get('certs/state').certs.length < MAX_CERTS}
                            addTitle={__("nfc.content.add_tag_title")}
                            addMessage={__("nfc.content.add_tag_prefix") + API.get('certs/state').certs.length + __("nfc.content.add_tag_infix") + MAX_CERTS + __("nfc.content.add_tag_suffix")}
                            onAddStart={async () => this.setState({addCert: {id: 0, name: "", cert: "", file: null}})}
                            onAddGetRows={() => [
                                {
                                    name: __("nfc.content.add_tag_tag_id"),
                                    value: <InputText value={state.addCert.name}
                                                    onValue={(v) => this.setState({addCert: {...state.addCert, name: v}})}
                                                    maxLength={32}
                                                    invalidFeedback={__("nfc.content.tag_id_invalid_feedback")}
                                                    required/>
                                },
                                {
                                    name: __("nfc.content.add_tag_tag_type"),
                                    value: <InputSelect items={[
                                            ["0","0"],
                                            ["1","1"],
                                            ["2","2"],
                                            ["3","3"],
                                            ["4","4"],
                                            ["5","5"],
                                            ["6","6"],
                                            ["7","7"],
                                        ]}
                                        value={state.addCert.id.toString()}
                                        onValue={(v) => this.setState({addCert: {...state.addCert, id: parseInt(v)}})}
                                        />
                                },
                                {
                                    name: "Select cert",
                                    value: <div class="custom-file">
                                                <input type="file" class="custom-file-input form-control" accept={"application/pem-certificate-chain"}
                                                    onChange={(ev) => this.setState({addCert: {...state.addCert, file: (ev.target as HTMLInputElement).files[0]}})}/>
                                                <label class="custom-file-label form-control rounded-left"
                                                    data-browse={"browse"}>{state.addCert.file ? state.addCert.file.name : "select file"}</label>
                                            </div>
                                }
                            ]}
                            onAddCommit={async () => {
                                //TODO size check
                                await API.call('certs/add', {
                                    id: state.addCert.id,
                                    name: state.addCert.name,
                                    cert: await state.addCert.file.text()
                                }, "error_string");

                                this.setState({
                                    addCert: {id: 0, name: "",  cert: "", file: null}
                                });
                            }}
                            onAddAbort={async () => this.setState({addCert: {id: 0, name: "",  cert: "", file: null}})} />
                    </div>
            </SubPage>
        )
    }
}

render(<Certs/>, $('#certs')[0])

export function init() {}

export function add_event_listeners(source: API.APIEventTarget) {}

export function update_sidebar_state(module_init: any) {
    $('#sidebar-certs').prop('hidden', !module_init.certs);
}
