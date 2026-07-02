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

#pragma once

#include <stdint.h>
#include <WString.h>

#include "modules/cm_networking/generated/cm_auth_type.enum.h"

class Config;

// Transport-neutral copy of the last authentication attempts seen on a charger.
// Layout-compatible with cm_auth_info (checked via static_asserts in
// cm_charger_backend.cpp) so that the CM backend can memcpy state packets.
struct ChargerAuthInfo {
    uint8_t _padding;
    CMAuthType auth_method;
    uint16_t last_seen_s;

    // Tagged by auth_method
    union {
        struct {
            uint8_t tag_type;
            uint8_t tag_id_len;
            uint8_t tag_id[10];
        } nfc;
        struct {
            uint8_t mac[6];
        } ev;
    };
};

// Transport-neutral snapshot of a remote charger's state, produced by a
// charger backend and consumed by update_charger_state() (current_allocator.cpp).
//
// charger_state uses the WARP semantics that the allocation algorithm expects:
// 0 - no vehicle connected
// 1 - connected, blocked (either by the charge management slot or another slot)
// 2 - connected, waiting for the vehicle to start charging
// 3 - charging
// 4 - error
struct ChargerRemoteState {
    // Must strictly increase between updates. Used to detect stale states:
    // if a backend reports the same uptime twice, the charger controller hangs
    // or the communication is broken.
    uint32_t uptime;

    // Unique identifier of the charger (ESP32 UID for WARP chargers, serial
    // number for third-party chargers). 0 if (not yet) known.
    uint32_t uid;

    // Milliseconds since the car stopped charging by itself in this charging
    // session, 0 if it did not.
    uint32_t car_stopped_charging;

    // Milliseconds since the last charger state change. Gates deriving the
    // requested current from the measured line currents. Only used if
    // time_since_state_change_known is true.
    uint32_t time_since_state_change;

    // Measured line currents in ampere. NAN if unknown.
    // Note that for compatibility with older WARP firmwares a value of 0 is
    // also treated as "no meter connected".
    float line_currents[3];

    // Measured total power in watt. NAN if unknown.
    float power_total;

    // Absolute energy in kilowatt-hours. NAN if unknown.
    float energy_abs;

    // Currently active charging current limit of the charger in milliampere.
    uint16_t allowed_current;

    // Maximum current supported by the charger in milliampere. For WARP
    // chargers a value of 0 means that a slot other than the charge management
    // slot blocks charging.
    uint16_t supported_current;

    uint8_t charger_state;
    uint8_t error_state;

    // Phases connected to the charger (1 or 3) if phases_known is true.
    uint8_t phases;

    // False if the backend cannot report phase information. The charger is
    // then assumed to charge three-phase without phase switch support.
    bool phases_known;

    bool time_since_state_change_known;
    bool phase_switch_supported;
    bool currently_switching_phases;
    bool meter_supported;
    bool cp_disconnect_supported;
    bool cp_disconnect_state;
};

// Transport-neutral allocation command, computed by the charge manager's send
// task and translated to the charger's protocol by the backend.
struct ChargerCommand {
    // Allocated charging current in milliampere. 0 blocks charging.
    uint16_t allocated_current;

    // Allocated phases (0 to 3). 0 blocks charging.
    int8_t allocated_phases;

    // True if the charger shall permanently disconnect the control pilot.
    bool cp_disconnect;

    // True if the charge manager has just (re)booted and the charger shall
    // keep its last known allocation instead of applying this command.
    bool ignore_allocation;
};

// Control interface between the charge manager and one managed charger.
// Implemented once per configured charger by a transport-specific backend
// (CM/UDP for WARP chargers, Modbus/TCP for third-party chargers, ...).
//
// A backend pushes received charger state into the charge manager via
// charge_manager.ingest_remote_state().
class IChargerBackend {
public:
    virtual ~IChargerBackend() = default;

    // Send the current allocation to the charger. Called round-robin by the
    // charge manager's send task. Return false if the command could not be
    // handed off (the send task will then retry this charger next tick).
    virtual bool send_update(const ChargerCommand &cmd) = 0;

    // Called if no state update was received from this charger for
    // RE_RESOLVE_TIMEOUT or if it never sent a state at all. Backends should
    // re-resolve the hostname, force a reconnect or similar.
    virtual void notify_unresponsive() = 0;
};

// Factory for charger backends of one charger class. Register with
// charge_manager.register_charger_generator() in pre_setup().
class IChargerBackendGenerator {
public:
    virtual ~IChargerBackendGenerator() = default;

    // Config prototype for the "ctrl" union member of this charger class.
    // Return Config::Null() if this class needs no extra configuration.
    virtual const Config *get_ctrl_config_prototype() = 0;

    // Create a backend for the charger at index idx of the charge manager's
    // charger arrays. host points to a persistent string; ctrl_config is the
    // active "ctrl" union member of this charger's config entry.
    virtual IChargerBackend *new_charger(uint8_t idx, const char *host, const Config *ctrl_config) = 0;

    // Validate the "ctrl" union member of a charger config entry of this
    // class. Return an error message or an empty string.
    virtual String validate_ctrl_config(const Config *ctrl_config) { (void)ctrl_config; return ""; }

    // Called once after all charger backends have been created.
    virtual void setup_chargers_done() {}
};
