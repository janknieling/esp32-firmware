/** @jsxImportSource preact */
import { h } from "preact";
let x = {
    "chargers_modbus_tcp": {
        "content": {
            "table_none": "Keine Registertabelle",
            "table_keba_p30": "KEBA P30 (Modbus TCP)",
            "table_custom": "Modbus/TCP (eigene Registertabelle)",

            "port": "Port",
            "port_muted": "typischerweise 502",
            "device_address": "Geräteadresse",
            "device_address_muted": "Modbus-Unit-ID, KEBA: 255",
            "phases": "Phasenanschluss",
            "phases_help": <><p>Anzahl der Phasen, mit denen die Wallbox angeschlossen ist. Wird die Phasenumschaltung verwendet, dient dieser Wert nur als Ausgangswert; der tatsächliche Zustand wird über das Phasenumschaltungs-Register gelesen.</p></>,
            "phases_1": "einphasig",
            "phases_3": "dreiphasig",
            "phase_switch": "Phasenumschaltung",
            "phase_switch_desc": "Der Lastmanager darf zwischen ein- und dreiphasigem Laden umschalten (KEBA P30 x-series mit Umschaltoption erforderlich)",

            "registers": "Registertabelle",
            "registers_help": <>
                <p>Jedes Register bekommt eine Rolle zugewiesen, über die das Lastmanagement den Wert interpretiert. Die Wertesemantik der Rollen entspricht der KEBA-P30-Modbus-Dokumentation (z.&nbsp;B. Ladestatus: 0&nbsp;Start, 1&nbsp;nicht bereit, 2&nbsp;bereit, 3&nbsp;lädt, 4&nbsp;Fehler, 5&nbsp;unterbrochen).</p>
                <p>Einheiten nach Anwendung von Offset und Skalierungsfaktor: Ströme in mA, Spannungen in V, Leistung in W, Energie in Wh.</p>
                <p>Erforderlich sind mindestens die Rollen <strong>Ladestatus</strong> und <strong>Ladestrom setzen</strong> (oder <strong>Freigabe setzen</strong>).</p>
            </>,
            "register_role": "Rolle",
            "register_role_help": <><p>Bestimmt, wie das Lastmanagement den Registerwert interpretiert. Schreibende Rollen werden als einzelnes Holding-Register (Funktionscode 6) geschrieben.</p></>,
            "register_roles_readable": "Lesend",
            "register_roles_writable": "Schreibend",
            "register_type": "Registertyp",
            "register_type_holding": "Holding-Register",
            "register_type_input": "Input-Register",
            "register_address": "Registeradresse",
            "register_address_muted": "0-basiert",
            "register_value_type": "Wertetyp",
            "register_offset": "Offset",
            "register_scale": "Skalierungsfaktor",
            "register_scale_help": <><p>Wert = (Rohwert + Offset) × Skalierungsfaktor. Beispiel: Liefert die Wallbox die Leistung in mW, ergibt ein Skalierungsfaktor von 0,001 die erwartete Einheit W.</p></>,
            "register_row": /*SFN*/(role: string, addr: number) => `${role} @ ${addr}`/*NF*/,
            "register_add_title": "Register hinzufügen",
            "register_add_message": /*SFN*/(have: number, max: number) => `${have} von ${max} Registern konfiguriert`/*NF*/,
            "register_edit_title": "Register bearbeiten",

            "role_1": "Ladestatus",
            "role_2": "Kabelstatus",
            "role_3": "Fehlercode",
            "role_4": "Ladestrom L1",
            "role_5": "Ladestrom L2",
            "role_6": "Ladestrom L3",
            "role_7": "Spannung L1",
            "role_8": "Spannung L2",
            "role_9": "Spannung L3",
            "role_10": "Wirkleistung",
            "role_11": "Gesamtenergie",
            "role_12": "Energie der Ladesitzung",
            "role_13": "Maximaler Ladestrom (aktiv)",
            "role_14": "Maximal unterstützter Strom",
            "role_15": "Phasenumschaltungsstatus",
            "role_16": "Seriennummer",
            "role_17": "Firmwareversion",
            "role_18": "Ladestrom setzen",
            "role_19": "Freigabe setzen",
            "role_20": "Phasenumschaltung auslösen",
            "role_21": "Phasenumschaltungsquelle",
            "role_22": "Failsafe-Strom",
            "role_23": "Failsafe-Timeout",
            "role_24": "Failsafe speichern",
            "role_25": "Stecker entriegeln",
            "role_26": "Energielimit setzen"
        }
    }
}
