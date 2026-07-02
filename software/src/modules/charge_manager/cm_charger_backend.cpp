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

#include "cm_charger_backend.h"

#include <string.h>
#include <time.h>
#include <limits>
#include <type_traits>

#include "event_log_prefix.h"
#include "generated/module_dependencies.h"

// The ChargeManager class and the charge_manager global come in via
// module_dependencies.h (Charge Manager references itself as a dependency).
#include "TFTools/Micros.h"
#include "bindings/base58.h"
#include "charge_manager_private.h"
#include "modules/cm_networking/cm_networking_defs.h"
#include "modules/cm_networking/generated/cm_auth_feedback.enum.h"
#include "modules/cm_networking/generated/client_error.enum.h"
#include "tools.h"

// ChargerAuthInfo is a transport-neutral copy of cm_auth_info. Verify that the
// layouts actually match, as state packets are memcpy'd below.
static_assert(sizeof(ChargerAuthInfo) == sizeof(cm_auth_info));
static_assert(offsetof(ChargerAuthInfo, auth_method) == offsetof(cm_auth_info, auth_method));
static_assert(offsetof(ChargerAuthInfo, last_seen_s) == offsetof(cm_auth_info, last_seen_s));
static_assert(offsetof(ChargerAuthInfo, nfc) == offsetof(cm_auth_info, nfc));
static_assert(offsetof(ChargerAuthInfo, ev) == offsetof(cm_auth_info, ev));

static const uint8_t *charger_idx_by_client_id = nullptr;

static uint16_t get_user_current(uint8_t user_id)
{
#if MODULE_USERS_AVAILABLE()
    return users.get_user_current(user_id);
#else
    return 32000;
#endif
}

static void update_charge_tracking(uint8_t idx, cm_state_v1 *v1)
{
#if MODULE_CHARGE_TRACKER_AVAILABLE()
    if (!charge_manager.central_management_enabled())
        return;

    auto now = now_us();
    auto &target = *charge_manager.get_mutable_charger_state(idx);

    // Populate first and last tracked charge when we first see this charger
    if (target.last_update == 0_us) {
        char uid_str[15];
        bool is_local_charger = false;
        if (target.uid == esp32_common.get_uid_num()) {
            is_local_charger = true;
        } else {
            tf_base58_encode(target.uid, uid_str);
        }

        ChargeStart cs;

        if (charge_tracker.currentlyCharging(is_local_charger ? nullptr : uid_str, &cs)) {
            if (v1->charger_state != 0 && (target.authenticated_user_id < 0 || target.authenticated_user_id == cs.user_id)) {
                // Car is still connected and we trust our charge tracking. Authenticate for tracked user.
                target.authenticated_user_id = cs.user_id;
                target.user_current = get_user_current(target.authenticated_user_id);
            } else {
                // Car not connected or we've already authenticated another user than was tracked the last time.
                // Track stop (and potentially track start below for other user)
                charge_tracker.endCharge(
                    0, // We don't know when the car was disconnected -> unknown duration
                    v1->energy_abs,
                    is_local_charger ? nullptr : uid_str);
            }
        }
    }

    // If tracking, a vehicle is plugged in and we are authenticated, track a start if not done already.
    if (v1->charger_state != 0 && target.authenticated_user_id != NOT_AUTHORIZED && target.authenticated_user_id != UNKNOWN_NFC_TAG) {
        char uid_str[15];
        bool is_local_charger = false;
        if (target.uid == esp32_common.get_uid_num()) {
            is_local_charger = true;
        } else {
            tf_base58_encode(target.uid, uid_str);
        }

        // TODO: optimize this!
        if (!charge_tracker.currentlyCharging(is_local_charger ? nullptr : uid_str)) {
            uint32_t charge_start = 0;
            timeval _timeval;
            if (rtc.clock_synced(&_timeval))
                charge_start = rtc.timestamp_minutes();

            charge_tracker.startCharge(charge_start,
                                       v1->energy_abs,
                                       target.authenticated_user_id,
                                       v1->evse_uptime,
                                       CMAuthType::None,
                                       Config::ConfVariant(),
                                       is_local_charger ? nullptr : uid_str);
        }
    }

    // If tracking and no vehicle is plugged in track a stop if not done already.
    if (v1->charger_state == 0) {
        char uid_str[15];
        bool is_local_charger = false;
        if (target.uid == esp32_common.get_uid_num()) {
            is_local_charger = true;
        } else {
            tf_base58_encode(target.uid, uid_str);
        }

        if (charge_tracker.currentlyCharging(is_local_charger ? nullptr : uid_str)) {
            charge_tracker.endCharge(
                (now - target.last_plug_in).to<seconds_t>().as<uint32_t>(),
                v1->energy_abs,
                is_local_charger ? nullptr : uid_str);
        }
    }
#endif
}

static bool on_auth_success(uint8_t new_user_id, ChargerState &target)
{
    if (target.authenticated_user_id == NOT_AUTHORIZED || target.authenticated_user_id == UNKNOWN_NFC_TAG) {
        // Authorize
        target.authenticated_user_id = new_user_id;
        target.user_current = get_user_current(target.authenticated_user_id);
        return true;
    }

    if (target.authenticated_user_id != new_user_id) {
        // Only allow deauthorize if this is the same user as the one that is authorized.
        return false;
    }

    // Deauthorize
    target.authenticated_user_id = NOT_AUTHORIZED;
    target.user_current = get_user_current(target.authenticated_user_id);
    return true;
}

static void update_authentication(uint8_t idx, cm_state_v1 *v1, cm_state_v5 *v5)
{
    auto &target = *charge_manager.get_mutable_charger_state(idx);

    if (v5 == nullptr) {
        memset(target.auth_info, 0, sizeof(target.auth_info));
    } else {
        memcpy(target.auth_info, v5->auth_info, sizeof(target.auth_info));
    }

    micros_t deadtime = 30_s;
#if MODULE_NFC_AVAILABLE()
    deadtime = nfc.get_deadtime_post_start();
#endif

    // If central auth is disabled, always authorize
    if (!charge_manager.central_management_enabled()) {
        target.authenticated_user_id = AUTHD_ANONYMOUSLY;
        target.user_current = get_user_current(target.authenticated_user_id);
        return;
    }

    // De-authorize on plug out
    if (v1->charger_state == 0 && target.charger_state != 0) {
        target.authenticated_user_id = NOT_AUTHORIZED;
        target.user_current = 0;

        // Allow re-auth immediately
        target.last_auth_success_timestamp = -deadtime;
    }

    // De-authorize when the last auth expires and there is still no car connected
    if (target.last_auth_success_timestamp != 0_us && deadline_elapsed(target.last_auth_success_timestamp + 30_s) && v1->charger_state == 0) {
        target.authenticated_user_id = NOT_AUTHORIZED;
        target.user_current = 0;
    }

    // If we are still in the auth deadtime, ignore new auths.
    if (target.last_auth_success_timestamp != 0_us && !deadline_elapsed(target.last_auth_success_timestamp + deadtime)) {
        return;
    }

    if (v5 == nullptr)
        return;

    micros_t latest_auth_fail = 0_us;
    for (int i = ARRAY_SIZE(v5->auth_info) - 1; i >= 0; --i) {
        const cm_auth_info &info = v5->auth_info[i];
        if (info.last_seen_s != 0 && info.last_seen_s < 2) {
            int16_t tag_auth = NOT_AUTHORIZED;
#if MODULE_CHARGE_AUTHORIZATION_AVAILABLE()
            tag_auth = charge_authorization.find_user(info);
            if (tag_auth == -1)
                tag_auth = UNKNOWN_NFC_TAG;
#endif
            if (tag_auth >= AUTHD_ANONYMOUSLY && on_auth_success((uint8_t) tag_auth, target)) {
                target.last_auth_success_timestamp = now_us() - seconds_t{info.last_seen_s};
                break; // A successful auth wins immediately.
            } else if (tag_auth == UNKNOWN_NFC_TAG) {
                latest_auth_fail = now_us() - seconds_t{info.last_seen_s};
            }
        }
    }

    if (latest_auth_fail != 0_us) {
        target.last_auth_fail_timestamp = latest_auth_fail;
    }
}

static void update_charge_mode(uint8_t idx, cm_state_v1 *v1, cm_state_v4 *v4)
{
    auto &target = *charge_manager.get_mutable_charger_state(idx);

    // If requested_charge_mode is default, no charge mode change is requested.
    if (v4 != nullptr && v4->requested_charge_mode != (uint8_t)ConfigChargeMode::Default)
        target.charge_mode = charge_manager.config_cm_to_cm((ConfigChargeMode)v4->requested_charge_mode);

    // If the car is unplugged, change back to the default charge mode once.
    if (v1->charger_state == 0 && target.charger_state != 0) {
        target.charge_mode = charge_manager.config_cm_to_cm(ConfigChargeMode::Default);
    }
}

static ChargerRemoteState cm_state_to_remote_state(cm_state_v1 *v1, cm_state_v2 *v2, cm_state_v3 *v3)
{
    ChargerRemoteState rs;

    rs.uptime = v1->evse_uptime;
    rs.uid = v1->esp32_uid;
    rs.car_stopped_charging = v1->car_stopped_charging;
    rs.time_since_state_change_known = v2 != nullptr;
    rs.time_since_state_change = v2 != nullptr ? v2->time_since_state_change : 0;

    for (size_t i = 0; i < ARRAY_SIZE(rs.line_currents); ++i)
        rs.line_currents[i] = v1->line_currents[i];

    rs.power_total = v1->power_total;
    rs.energy_abs = v1->energy_abs;
    rs.allowed_current = v1->allowed_charging_current;
    rs.supported_current = v1->supported_current;
    rs.charger_state = v1->charger_state;
    rs.error_state = v1->error_state;

    rs.phases_known = v3 != nullptr;
    if (v3 != nullptr) {
        rs.phases = CM_STATE_V3_PHASES_CONNECTED_GET(v3->phases);
        rs.phase_switch_supported = CM_STATE_V3_CAN_PHASE_SWITCH_IS_SET(v3->phases);
        rs.currently_switching_phases = CM_STATE_V3_CURRENTLY_SWITCHING_IS_SET(v3->phases);
    } else {
        rs.phases = 3;
        rs.phase_switch_supported = false;
        rs.currently_switching_phases = false;
    }

    rs.meter_supported = CM_FEATURE_FLAGS_METER_IS_SET(v1->feature_flags);
    rs.cp_disconnect_supported = CM_FEATURE_FLAGS_CP_DISCONNECT_IS_SET(v1->feature_flags);
    rs.cp_disconnect_state = CM_STATE_FLAGS_CP_DISCONNECTED_IS_SET(v1->state_flags);

    return rs;
}

const Config *CMChargerBackendGenerator::get_ctrl_config_prototype()
{
    return Config::Null();
}

IChargerBackend *CMChargerBackendGenerator::new_charger(uint8_t idx, const char *host, const Config * /*ctrl_config*/)
{
    uint8_t cm_client_id = (uint8_t)this->cm_hosts.size();

    this->cm_hosts.push_back(host);
    this->charger_indices.push_back(idx);

    return new CMChargerBackend(idx, cm_client_id);
}

void CMChargerBackendGenerator::setup_chargers_done()
{
    if (this->cm_hosts.size() == 0)
        return;

    // The vectors are not modified anymore, so their data pointers stay valid.
    charger_idx_by_client_id = this->charger_indices.data();

    cm_networking.register_manager(this->cm_hosts.data(), this->cm_hosts.size(), [](uint8_t client_id, cm_state_v1 *v1, cm_state_v2 *v2, cm_state_v3 *v3, cm_state_v4 *v4, cm_state_v5 *v5) {
        uint8_t idx = charger_idx_by_client_id[client_id];

        ChargerRemoteState rs = cm_state_to_remote_state(v1, v2, v3);

        bool accepted = charge_manager.ingest_remote_state(idx, rs, [idx, v1, v4, v5]() {
            // These run after the staleness check and UID update, but before
            // update_charger_state(), because they compare the new charger
            // state against the not-yet-updated ChargerState.
            update_charge_mode(idx, v1, v4);
            update_authentication(idx, v1, v5);
            update_charge_tracking(idx, v1);
        });

        if (!accepted)
            return;

        if (CM_FEATURE_FLAGS_URGENT_IS_SET(v1->feature_flags))
            charge_manager.request_urgent_send(idx);

        if (CM_FEATURE_FLAGS_REQUEST_REALLOCATION_IS_SET(v1->feature_flags))
            charge_manager.trigger_allocator_run(false);
    }, [](uint8_t client_id, ClientError error) {
        static_assert(std::is_same_v<std::underlying_type_t<ClientError>, std::underlying_type_t<CASError>>);
        static_assert(ClientError::_min == ClientError::OK);
        static_assert(ClientError::_max == ClientError::NotManaged);
        static_assert(to_underlying(ClientError::_max) - to_underlying(ClientError::_min) == 3);
        static_assert(to_underlying(ClientError::OK) == to_underlying(CASError::OK));
        static_assert(to_underlying(ClientError::InvalidHeader) == to_underlying(CASError::InvalidHeader));
        static_assert(to_underlying(ClientError::NotManaged) == to_underlying(CASError::NotManaged));

        charge_manager.ingest_client_error(charger_idx_by_client_id[client_id], (CASError)error);
    });
}

bool CMChargerBackend::send_update(const ChargerCommand &cmd)
{
    auto current = cmd.allocated_current;
    auto cp_disconnect = cmd.cp_disconnect;
    auto phases = cmd.allocated_phases;

    auto &charger = *charge_manager.get_mutable_charger_state(this->charger_idx);
    auto charge_mode = charge_manager.cm_to_config_cm(charger.charge_mode);

    CMAuthFeedback auth_feedback = CMAuthFeedback::None;
    if (charge_manager.central_management_enabled()) {
        if (!deadline_elapsed(charger.last_auth_success_timestamp + 2_s)) {
            auth_feedback = CMAuthFeedback::Ack;
        } else if (!deadline_elapsed(charger.last_auth_fail_timestamp + 2_s)) {
            auth_feedback = CMAuthFeedback::Nack;
        } else if (charger.charger_state == 1 && charger.authenticated_user_id == NOT_AUTHORIZED) {
            auth_feedback = CMAuthFeedback::Nag;
        }
    }

    if (cmd.ignore_allocation) {
        // This requires managed chargers to support cm_command_v3!
        charge_mode = ConfigChargeMode::Default; // This will instruct the managed charger to send its charge mode back as if it wants to request a charge mode change

        // Set sane defaults for managed chargers with older firmwares
        current = std::numeric_limits<decltype(current)>::max(); // Directly passed through to the EVSE bricklet, which ignores values > 32000
        cp_disconnect = false; // Way more likely to be correct than that the manager restarted while a phase switch was in progress
        phases = 0; // 0 phases are ignored except with WARP* firmware == 2.6.0. 2.6.1 fixed this two weeks later
    }

    return cm_networking.send_manager_update(this->cm_client_id,
                                             cmd.ignore_allocation,
                                             current,
                                             cp_disconnect,
                                             phases,
                                             charge_mode,
                                             charge_manager.get_supported_charge_mode_bitmask(),
                                             auth_feedback,
                                             charge_manager.central_management_enabled(),
                                             charge_manager.central_management_enabled());
}

void CMChargerBackend::notify_unresponsive()
{
    cm_networking.notify_charger_unresponsive(this->cm_client_id);
}
