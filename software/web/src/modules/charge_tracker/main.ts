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

import feather from "../../ts/feather";

declare function __(s: string): string;

// Creates a date and time string that will be understood by Excel, Libreoffice, etc.
// (At least for de and en locales)
function timestamp_min_to_date(timestamp_minutes: number) {
    if (timestamp_minutes == 0) {
        return __("charge_tracker.script.unknown_charge_start");
    }
    let date_fmt: any = { year: 'numeric', month: '2-digit', day: '2-digit'};
    let time_fmt: any = {hour: '2-digit', minute:'2-digit' };
    let fmt = Object.assign({}, date_fmt, time_fmt);

    let date = new Date(timestamp_minutes * 60000);
    let result = date.toLocaleString([], fmt);

    let date_result = date.toLocaleDateString([], date_fmt);
    let time_result = date.toLocaleTimeString([], time_fmt);

    // By default there is a comma between the date and time part of the string.
    // This comma (even if the whole date is marked as string for CSV) prevents office programs
    // to understand that this is a date.
    // Remove this (and only this) comma without assuming anything about the localized string.
    if (result == date_result + ", " + time_result) {
        return date_result + " " + time_result;
    }
    if (result == time_result + ", " + date_result) {
        return time_result + " " + date_result;
    }

    return result;
}

function update_last_charges() {
    let charges = API.get('charge_tracker/last_charges');
    let users_config = API.get('users/config');

    $('#charge_tracker_last_charges').html(charges.map((user) => {
            let display_name = __("charge_tracker.script.unknown_user")

            if (user.user_id != 0) {
                display_name = __("charge_tracker.script.deleted_user")
                let filtered = users_config.users.filter(x => x.id == user.user_id);
                if (filtered.length == 1)
                    display_name = filtered[0].display_name
            }

            return `<div class="list-group-item">
            <div class="row">
                <div class="col">
                    <div class="mb-2"><span class="mr-1" data-feather="user"></span><span style="vertical-align: middle;">${display_name}</span></div>
                    <div><span class="mr-1" data-feather="calendar"></span><span style="vertical-align: middle;">${timestamp_min_to_date(user.timestamp_minutes)}</span></div>
                </div>
                <div class="col-auto">
                    <div class="mb-2"><span class="mr-1" data-feather="battery-charging"></span><span style="vertical-align: middle;">${user.energy_charged === null ? "N/A" : util.toLocaleFixed(user.energy_charged, 3)} kWh</span></div>
                    <div><span class="mr-1" data-feather="clock"></span><span style="vertical-align: middle;">${util.format_timespan(user.charge_duration)}</span></div>
                </div>
            </div>
            </div>`
        }).join(""));
    feather.replace();
}

function to_csv_line(vals: string[]) {
    let line = vals.map(entry => '"' + entry.replace(/\"/, '""') + '"');

    return line.join(",") + "\r\n";
}

async function downloadChargeLog() {
    let users: string[] = [];

    await fetch('/users/all_usernames')
        .then(response => response.arrayBuffer())
        .then(buffer => {
            if (buffer.byteLength != 256 * 32) {
                console.log("Unexpected length of all_usernames!");
                return;
            }

            const decoder = new TextDecoder();
            for(let i = 0; i < 256; ++i) {
                let view = new DataView(buffer, i * 32, 32);
                users.push(decoder.decode(view));
            }
        })
        .catch(err => console.log(err));

    await fetch('/charge_tracker/charge_log')
        .then(response => response.arrayBuffer())
        .then(buffer => {
            let line = [
                __("charge_tracker.script.csv_header_start"),
                __("charge_tracker.script.csv_header_user"),
                __("charge_tracker.script.csv_header_energy"),
                __("charge_tracker.script.csv_header_duration"),
                "",
                __("charge_tracker.script.csv_header_meter_start"),
                __("charge_tracker.script.csv_header_meter_end"),
            ];

            let result = to_csv_line(line);
            let users_config = API.get('users/config');

            for(let i = 0; i < buffer.byteLength; i += 16) {
                let view = new DataView(buffer, i, 16);

                let timestamp_minutes = view.getUint32(0, true);
                let meter_start = view.getFloat32(4, true);
                let user_id = view.getUint8(8);
                let charge_duration = view.getUint32(9, true) & 0x00FFFFFF;
                let meter_end = view.getFloat32(12, true);

                let filtered = users_config.users.filter(x => x.id == user_id);

                let display_name = "";
                if (user_id == 0)
                    display_name = __("charge_tracker.script.unknown_user");
                else if (filtered.length == 1)
                    display_name = filtered[0].display_name
                else
                    display_name = users[user_id];

                let line = [
                    timestamp_min_to_date(timestamp_minutes),
                    display_name,
                    (Number.isNaN(meter_start) || Number.isNaN(meter_end)) ? 'N/A' : util.toLocaleFixed(meter_end - meter_start, 3),
                    charge_duration.toString(),
                    "",
                    (Number.isNaN(meter_start) || Number.isNaN(meter_end)) ? 'N/A' : util.toLocaleFixed(meter_start, 3),
                    (Number.isNaN(meter_start) || Number.isNaN(meter_end)) ? 'N/A' : util.toLocaleFixed(meter_end, 3)
                ];

                result += to_csv_line(line);
            }

            let t = (new Date()).toISOString().replace(/:/gi, "-").replace(/\./gi, "-");
            util.downloadToFile(result, "charge-log-" + t + ".csv", "text/csv; charset=utf-8; header=present");
        })
        .catch(err => console.log(err));
}

function update_current_charge() {
    let cc = API.get('charge_tracker/current_charge');
    let evse_ll = API.get('evse/low_level_state');
    let mv = API.get('meter/values');
    let uc = API.get('users/config');

    $('#charge_tracker_current_charge').prop("hidden", cc.user_id == -1);

    if (cc.user_id == -1) {
        return;
    }

    let user_display_name = uc.users.filter((x) => x.id == cc.user_id)[0].display_name;
    let energy_charged = mv.energy_abs - cc.meter_start;
    let time_charging = evse_ll.uptime - cc.evse_uptime_start
    if (evse_ll.uptime < cc.evse_uptime_start)
        time_charging += 0xFFFFFFFF;

    time_charging = Math.floor(time_charging / 1000);

    $('#users_status_charging_user').html(cc.user_id == 0 ? "unbekannter Nutzer" : user_display_name);
    $('#users_status_charging_time').html(util.format_timespan(time_charging));
    $('#users_status_charged_energy').html(cc.meter_start == null ? "N/A" : util.toLocaleFixed(energy_charged, 3) + " kWh");
    $('#users_status_charging_start').html(timestamp_min_to_date(cc.timestamp_minutes));
}

export function init() {
    $('#charge_tracker_download').on("click", () =>{
        $('#charge_tracker_download_spinner').prop("hidden", false);
        downloadChargeLog().finally(() => $('#charge_tracker_download_spinner').prop("hidden", true));
    });
}

export function addEventListeners(source: API.ApiEventTarget) {
    source.addEventListener('charge_tracker/last_charges', update_last_charges);

    source.addEventListener('charge_tracker/current_charge', update_current_charge);
    source.addEventListener('evse/low_level_state', update_current_charge);
    source.addEventListener('meter/values', update_current_charge);
    source.addEventListener('users/config', update_current_charge);
}

export function updateLockState(module_init: any) {
    $('#sidebar-charge_tracker').prop('hidden', !module_init.charge_tracker);
}
