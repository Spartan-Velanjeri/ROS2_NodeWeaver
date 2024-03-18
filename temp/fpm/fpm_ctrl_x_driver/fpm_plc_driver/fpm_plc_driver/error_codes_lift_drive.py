# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.
"""List of error codes for the lift drive.
"""
# black formatter is disabled (it will be ugly)
# fmt: off
# flake8: noqa
# FIXME (remove flake ignore F601 above when error codes implemented)
error_codes_lift_drive = {
    0x2300: {
        "Bedeutung": "Gruppe 31: I²t",
        "Anzeige": "31-x",
    },
    0x2311: {
        "Bedeutung": "I²t-Servoregler",
        "Anzeige": "31-1",
    },
    0x2312: {
        "Bedeutung": "I²t-Motor",
        "Anzeige": "31-0",
    },
    0x2313: {
        "Bedeutung": "I²t-PFC",
        "Anzeige": "31-2",
    },
    0x2314: {
        "Bedeutung": "I²t-Bremswiderstand",
        "Anzeige": "31-3",
    },
    0x2320: {
        "Bedeutung": "Gruppe 6: Kurzschluss Endstufe",
        "Anzeige": "6-x",
    },
    0x3200: {
        "Bedeutung": "Gruppe 32: PFC",
        "Anzeige": "32-x",
    },
    0x3210: {
        "Bedeutung": "Gruppe 7: Überspannung",
        "Anzeige": "7-x",
    },
    0x3220: {
        "Bedeutung": "Gruppe 2: Unterspannung im Zwischenkreis",
        "Anzeige": "2-x",
    },
    0x3280: {
        "Bedeutung": "Ladezeit Zwischenkreis überschritten",
        "Anzeige": "32-0",
    },
    0x3281: {
        "Bedeutung": "Unterspannung für aktive PFC",
        "Anzeige": "32-1",
    },
    0x3282: {
        "Bedeutung": "Überlast Bremschopper. Zwischenkreis konnte nicht entladen werden",
        "Anzeige": "32-5",
    },
    0x3283: {
        "Bedeutung": "Entladezeit Zwischenkreis überschritten",
        "Anzeige": "32-6",
    },
    0x3284: {
        "Bedeutung": "Leistungsversorgung für Reglerfreigabe fehlt",
        "Anzeige": "32-7",
    },
    0x3285: {
        "Bedeutung": "Ausfall Leistungsversorgung bei freigegebenem Servoregler",
        "Anzeige": "32-8",
    },
    0x3286: {
        "Bedeutung": "Phasenausfall",
        "Anzeige": "32-9",
    },
    0x4200: {
        "Bedeutung": "Gruppe 4: Übertemperatur",
        "Anzeige": "4-x",
    },
    0x4210: {
        "Bedeutung": "Übertemperatur Leistungsteil",
        "Anzeige": "4-0",
    },
    0x4280: {
        "Bedeutung": "Übertemperatur Zwischenkreis",
        "Anzeige": "4-1",
    },
    0x4310: {
        "Bedeutung": "Gruppe 3: Übertemperatur Motor",
        "Anzeige": "3-x",
    },
    0x5080: {
        "Bedeutung": "Gruppe 90: HW-Initialisierung",
        "Anzeige": "90-x",
    },
    0x5110: {
        "Bedeutung": "Gruppe 5: Interne Spannungsversorgung",
        "Anzeige": "5-x",
    },
    0x5114: {
        "Bedeutung": "Ausfall interne Spannung 1",
        "Anzeige": "5-0",
    },
    0x5115: {
        "Bedeutung": "Ausfall interne Spannung 2",
        "Anzeige": "5-1",
    },
    0x5116: {
        "Bedeutung": "Ausfall Treiberversorgung",
        "Anzeige": "5-2",
    },
    0x5200: {
        "Bedeutung": "Gruppe 21: Strommessung",
        "Anzeige": "21-x",
    },
    0x5220: {
        "Bedeutung": "Unerwarteter Hardware-Fehler",
        "Anzeige": "16-4",
    },
    0x5280: {
        "Bedeutung": "Fehler 1 Strommessung U",
        "Anzeige": "21-0",
    },
    0x5281: {
        "Bedeutung": "Fehler 1 Strommessung V",
        "Anzeige": "21-1",
    },
    0x5282: {
        "Bedeutung": "Fehler 2 Strommessung U",
        "Anzeige": "21-2",
    },
    0x5283: {
        "Bedeutung": "Fehler 2 Strommessung V",
        "Anzeige": "21-3",
    },
    0x5410: {
        "Bedeutung": "Unterspannung digitale I/Os",
        "Anzeige": "5-3",
    },
    0x5410: {
        "Bedeutung": "Überstrom digitale I/Os",
        "Anzeige": "5-4",
    },
    0x5430: {
        "Bedeutung": "Gruppe 24: Überwachung Analogeingang",
        "Anzeige": "24-x",
    },
    0x5500: {
        "Bedeutung": "Gruppe 26: Flash",
        "Anzeige": "26-x",
    },
    0x5580: {
        "Bedeutung": "Fehlender User-Parametersatz",
        "Anzeige": "26-0",
    },
    0x5581: {
        "Bedeutung": "Checksummenfehler",
        "Anzeige": "26-1",
    },
    0x5582: {
        "Bedeutung": "Flash: Fehler beim Schreiben",
        "Anzeige": "26-2",
    },
    0x5583: {
        "Bedeutung": "Flash: Fehler beim Löschen",
        "Anzeige": "26-3",
    },
    0x5584: {
        "Bedeutung": "Flash: Fehler im internen Flash",
        "Anzeige": "26-4",
    },
    0x5585: {
        "Bedeutung": "Fehlende Kalibrierdaten",
        "Anzeige": "26-5",
    },
    0x5586: {
        "Bedeutung": "Fehlende User-Positionsdatensätze",
        "Anzeige": "26-6",
    },
    0x6000: {
        "Bedeutung": "Gruppe 25: Ungültiger Gerätetyp",
        "Anzeige": "25-x",
    },
    0x6000: {
        "Bedeutung": "Gruppe 91: SW-Initialisierung",
        "Anzeige": "91-x",
    },
    0x6080: {
        "Bedeutung": "Ungültiger Gerätetyp",
        "Anzeige": "25-0",
    },
    0x6081: {
        "Bedeutung": "Gerätetyp nicht unterstützt",
        "Anzeige": "25-1",
    },
    0x6082: {
        "Bedeutung": "Hardware-Revision nicht unterstützt",
        "Anzeige": "25-2",
    },
    0x6083: {
        "Bedeutung": "Gerätefunktion beschränkt",
        "Anzeige": "25-3",
    },
    0x6100: {
        "Bedeutung": "Gruppe 16: Programmablauf",
        "Anzeige": "16-x",
    },
    0x6180: {
        "Bedeutung": "Gruppe 1: Stacküberlauf",
        "Anzeige": "1-x ",
    },
    0x6181: {
        "Bedeutung": "Programmausführung fehlerhaft",
        "Anzeige": "16-0",
    },
    0x6182: {
        "Bedeutung": "Illegaler Interrupt",
        "Anzeige": "16-1",
    },
    0x6183: {
        "Bedeutung": "Unerwarteter Zustand",
        "Anzeige": "16-3",
    },
    0x6184: {
        "Bedeutung": "Gruppe 15: Mathematik",
        "Anzeige": "15-x",
    },
    0x6185: {
        "Bedeutung": "Division durch Null",
        "Anzeige": "15-0",
    },
    0x6186: {
        "Bedeutung": "Bereichsüberschreitung",
        "Anzeige": "15-1",
    },
    0x6187: {
        "Bedeutung": "Initialisierungsfehler",
        "Anzeige": "16-2",
    },
    0x6188: {
        "Bedeutung": "Gruppe 82: Interne Ablaufsteuerung",
        "Anzeige": "82-x",
    },
    0x6320: {
        "Bedeutung": "Gruppe 36: Parameter",
        "Anzeige": "36-x",
    },
    0x6380: {
        "Bedeutung": "Gruppe 30: Interne Berechnungen",
        "Anzeige": "30-x",
    },
    0x7122: {
        "Bedeutung": "Gruppe 14: Motor- und Winkelgeber-Identifikation",
        "Anzeige": "14-x",
    },
    0x7300: {
        "Bedeutung": "Gruppe 8: Winkelgeber",
        "Anzeige": "8-x",
    },
    0x7380: {
        "Bedeutung": "Winkelgeberfehler Resolver/Hallgeber",
        "Anzeige": "8-0",
    },
    0x7382: {
        "Bedeutung": "Fehler Spursignale Z0 Inkrementalgeber",
        "Anzeige": "8-2",
    },
    0x7383: {
        "Bedeutung": "Fehler Spursignale Z1 Inkrementalgeber",
        "Anzeige": "8-3",
    },
    0x7384: {
        "Bedeutung": "Fehler Spursignale digitaler Inkrementalgeber",
        "Anzeige": "8-4",
    },
    0x7385: {
        "Bedeutung": "Fehler Hallgebersignale Inkrementalgeber",
        "Anzeige": "8-5",
    },
    0x7386: {
        "Bedeutung": "Kommunikationsfehler Winkelgeber",
        "Anzeige": "8-6",
    },
    0x7387: {
        "Bedeutung": "Leitfrequenzeingang: Signalamplitude Inkrementalspur fehlerhaft",
        "Anzeige": "8-7",
    },
    0x7388: {
        "Bedeutung": "Interner Winkelgeberfehler",
        "Anzeige": "8-8",
    },
    0x7389: {
        "Bedeutung": "Winkelgeber an [X2B/X6] wird nicht unterstützt",
        "Anzeige": "8-9",
    },
    0x73A0: {
        "Bedeutung": "Gruppe 9: Winkelgeber-Parametersatz",
        "Anzeige": "9-x",
    },
    0x73A1: {
        "Bedeutung": "Winkelgeber-Parametersatz: veraltetes Format",
        "Anzeige": "9-0",
    },
    0x73A2: {
        "Bedeutung": "Winkelgeber-Parametersatz kann nicht dekodiert werden",
        "Anzeige": "9-1",
    },
    0x73A3: {
        "Bedeutung": "Winkelgeber-Parametersatz: unbekannte Version",
        "Anzeige": "9-2",
    },
    0x73A4: {
        "Bedeutung": "Winkelgeber-Parametersatz: defekte Datenstruktur",
        "Anzeige": "9-3",
    },
    0x73A5: {
        "Bedeutung": "Schreibgeschütztes EEPROM Winkelgeber",
        "Anzeige": "9-7",
    },
    0x73A6: {
        "Bedeutung": "EEPROM Winkelgeber zu klein",
        "Anzeige": "9-9",
    },
    0x7580: {
        "Bedeutung": "Gruppe 60: Ethernet",
        "Anzeige": "60-x",
    },
    0x7581: {
        "Bedeutung": "Gruppe 61: Ethernet",
        "Anzeige": "61-x",
    },
    0x8000: {
        "Bedeutung": "Gruppe 45: Treiberversorgung IGBT",
        "Anzeige": "45-x",
    },
    0x8080: {
        "Bedeutung": "Gruppe 43: HW-Endschalter",
        "Anzeige": "43-x",
    },
    0x8081: {
        "Bedeutung": "Endschalter: Negativer Sollwert gesperrt",
        "Anzeige": "43-0",
    },
    0x8082: {
        "Bedeutung": "Endschalter: Positiver Sollwert gesperrt",
        "Anzeige": "43-1",
    },
    0x8083: {
        "Bedeutung": "Endschalter: Positionierung unterdrückt",
        "Anzeige": "43-2",
    },
    0x8084: {
        "Bedeutung": "Treiberversorgung nicht abschaltbar",
        "Anzeige": "45-0",
    },
    0x8085: {
        "Bedeutung": "Treiberversorgung nicht aktivierbar",
        "Anzeige": "45-1",
    },
    0x8086: {
        "Bedeutung": "Treiberversorgung wurde aktiviert",
        "Anzeige": "45-2",
    },
    0x8090: {
        "Bedeutung": "Gruppe 51: FSM 2.0",
        "Anzeige": "51-x",
    },
    0x8091: {
        "Bedeutung": "Kein / unbekanntes FSM-Modul oder Treiberversorgung fehlerhaft",
        "Anzeige": "51-0",
    },
    0x8093: {
        "Bedeutung": "FSM: Ungleicher Modultyp",
        "Anzeige": "51-2",
    },
    0x8094: {
        "Bedeutung": "FSM: Ungleiche Modulversion",
        "Anzeige": "51-3",
    },
    0x8095: {
        "Bedeutung": "FSM: Fehler in der SSIO-Kommunikation",
        "Anzeige": "51-4",
    },
    0x8096: {
        "Bedeutung": "FSM: Fehler in der Bremsenansteuerung",
        "Anzeige": "51-5",
    },
    0x8097: {
        "Bedeutung": "FSM: Ungleiche Modul-Seriennummer",
        "Anzeige": "51-6",
    },
    0x8098: {
        "Bedeutung": "Gruppe 52: FSM 2.0 STO",
        "Anzeige": "52-x",
    },
    0x8099: {
        "Bedeutung": "FSM: Diskrepanzzeit abgelaufen",
        "Anzeige": "52-1",
    },
    0x809A: {
        "Bedeutung": "FSM: Ausfall STOA/STOB bei freigegebener Endstufe",
        "Anzeige": "52-2",
    },
    0x809B: {
        "Bedeutung": "FSM: Fehler in den Begrenzungen",
        "Anzeige": "52-3",
    },
    0x80A0: {
        "Bedeutung": "Gruppe 53: FSM: Verletzung von Sicherheitsbedingungen",
        "Anzeige": "53-x",
    },
    0x80A1: {
        "Bedeutung": "USF0: Sicherheitsbedingung verletzt",
        "Anzeige": "53-0",
    },
    0x80A2: {
        "Bedeutung": "USF1: Sicherheitsbedingung verletzt",
        "Anzeige": "53-1",
    },
    0x80A3: {
        "Bedeutung": "USF2: Sicherheitsbedingung verletzt",
        "Anzeige": "53-2",
    },
    0x80A4: {
        "Bedeutung": "USF3: Sicherheitsbedingung verletzt",
        "Anzeige": "53-3",
    },
    0x80A9: {
        "Bedeutung": "Gruppe 54: FSM: Verletzung von Sicherheitsbedingungen",
        "Anzeige": "54-x",
    },
    0x80AA: {
        "Bedeutung": "SBC: Sicherheitsbedingung verletzt",
        "Anzeige": "54-0",
    },
    0x80AC: {
        "Bedeutung": "SS2: Sicherheitsbedingung verletzt",
        "Anzeige": "54-2",
    },
    0x80AD: {
        "Bedeutung": "SOS: Sicherheitsbedingung verletzt",
        "Anzeige": "54-3",
    },
    0x80AE: {
        "Bedeutung": "SS1: Sicherheitsbedingung verletzt",
        "Anzeige": "54-4",
    },
    0x80AF: {
        "Bedeutung": "STO: Sicherheitsbedingung verletzt",
        "Anzeige": "54-5",
    },
    0x80B0: {
        "Bedeutung": "SBC: Bremse > 24 h nicht gelüftet",
        "Anzeige": "54-6",
    },
    0x80B1: {
        "Bedeutung": "SOS: SOS > 24 h angefordert",
        "Anzeige": "54-7",
    },
    0x80C0: {
        "Bedeutung": "Gruppe 55: FSM: Istwerterfassung 1",
        "Anzeige": "55-x",
    },
    0x80C1: {
        "Bedeutung": "FSM: Kein Drehzahl-/Positionsistwert verfügbar oder Stillstand > 24 h",
        "Anzeige": "55-0",
    },
    0x80C2: {
        "Bedeutung": "FSM: SINCOS-Geber [X2B] - Fehler Spursignale",
        "Anzeige": "55-1",
    },
    0x80C3: {
        "Bedeutung": "FSM: SINCOS-Geber [X2B] - Stillstand > 24 h",
        "Anzeige": "55-2",
    },
    0x80C4: {
        "Bedeutung": "FSM: Resolver [X2A] - Signalfehler",
        "Anzeige": "55-3",
    },
    0x80C6: {
        "Bedeutung": "FSM: Sonstiger Geber [X2B] - Fehlerhafte Winkelinformation",
        "Anzeige": "55-7",
    },
    0x80C7: {
        "Bedeutung": "FSM: Unzulässige Beschleunigung detektiert",
        "Anzeige": "55-8",
    },
    0x80D0: {
        "Bedeutung": "Gruppe 56: FSM: Istwerterfassung 2",
        "Anzeige": "56-x",
    },
    0x80D1: {
        "Bedeutung": "FSM: Drehzahl- / Winkeldifferenz Geber 1 - 2",
        "Anzeige": "56-8",
    },
    0x80D2: {
        "Bedeutung": "FSM: Fehler Kreuzvergleich Geberauswertung",
        "Anzeige": "56-9",
    },
    0x80E0: {
        "Bedeutung": "Gruppe 57: FSM: Ein-/Ausgänge",
        "Anzeige": "57-x",
    },
    0x80E1: {
        "Bedeutung": "FSM: E/A - Fehler Selbsttest (intern/extern)",
        "Anzeige": "57-0",
    },
    0x80E2: {
        "Bedeutung": "FSM: Digitale Eingänge - Fehler Signalpegel",
        "Anzeige": "57-1",
    },
    0x80E3: {
        "Bedeutung": "FSM: Digitale Eingänge - Fehler Testimpuls",
        "Anzeige": "57-2",
    },
    0x80E7: {
        "Bedeutung": "FSM: Übertemperatur",
        "Anzeige": "57-6",
    },
    0x80E8: {
        "Bedeutung": "Gruppe 58: FSM: Kommunikation / Parametrierung",
        "Anzeige": "58-x",
    },
    0x80E9: {
        "Bedeutung": "FSM: Plausibilitätsprüfung Parameter",
        "Anzeige": "58-0",
    },
    0x80EA: {
        "Bedeutung": "FSM: Allgemeiner Fehler Parametrierung",
        "Anzeige": "58-1",
    },
    0x80ED: {
        "Bedeutung": "FSM: Puffer interne Kommunikation",
        "Anzeige": "58-4",
    },
    0x80EE: {
        "Bedeutung": "FSM: Kommunikation Sicherheitsmodul - Servoregler",
        "Anzeige": "58-5",
    },
    0x80EF: {
        "Bedeutung": "FSM: Fehler Kreuzvergleich Prozessoren 1 - 2",
        "Anzeige": "58-6",
    },
    0x80F0: {
        "Bedeutung": "Gruppe 59: FSM: Interne Fehler",
        "Anzeige": "59-x",
    },
    0x80F1: {
        "Bedeutung": "FSM: Failsafe-Versorgung / sichere Impulssperre",
        "Anzeige": "59-1",
    },
    0x80F2: {
        "Bedeutung": "FSM: Fehler externe Spannungsversorgung",
        "Anzeige": "59-2",
    },
    0x80F3: {
        "Bedeutung": "FSM: Fehler interne Spannungsversorgung",
        "Anzeige": "59-3",
    },
    0x80F4: {
        "Bedeutung": "FSM: Fehlermanagement: Zu viele Fehler",
        "Anzeige": "59-4",
    },
    0x80F5: {
        "Bedeutung": "FSM: Fehler beim Schreiben in den permanenten Ereignisspeicher",
        "Anzeige": "59-5",
    },
    0x80F6: {
        "Bedeutung": "FSM: Fehler beim Speichern des Parametersatzes",
        "Anzeige": "59-6",
    },
    0x80F7: {
        "Bedeutung": "FSM: Flash-Checksummenfehler",
        "Anzeige": "59-7",
    },
    0x80F8: {
        "Bedeutung": "FSM: Interne Überwachung Prozessor 1 - 2",
        "Anzeige": "59-8",
    },
    0x80F9: {
        "Bedeutung": "FSM: Sonstiger unerwarteter Fehler",
        "Anzeige": "59-9",
    },
    0x8100: {
        "Bedeutung": "Gruppe 12: CAN-Kommunikation",
        "Anzeige": "12-x",
    },
    0x8100: {
        "Bedeutung": "Gruppe 13: Timeout CAN-Bus",
        "Anzeige": "13-x",
    },
    0x8120: {
        "Bedeutung": "CAN: Kommunikationsfehler, Bus AUS",
        "Anzeige": "12-1",
    },
    0x8130: {
        "Bedeutung": "CAN: Node Guarding",
        "Anzeige": "12-4",
    },
    0x8180: {
        "Bedeutung": "CAN: Knotennummer doppelt",
        "Anzeige": "12-0",
    },
    0x8181: {
        "Bedeutung": "CAN: Kommunikationsfehler beim Senden",
        "Anzeige": "12-2",
    },
    0x8182: {
        "Bedeutung": "CAN: Kommunikationsfehler beim Empfangen",
        "Anzeige": "12-3",
    },
    0x8183: {
        "Bedeutung": "CAN: Protokollfehler",
        "Anzeige": "12-9",
    },
    0x8184: {
        "Bedeutung": "Timeout CAN-Bus",
        "Anzeige": "13-0",
    },
    0x8200: {
        "Bedeutung": "Gruppe 50: CAN-Kommunikation",
        "Anzeige": "50-x",
    },
    0x8210: {
        "Bedeutung": "CAN: RPDO zu kurz",
        "Anzeige": "12-5",
    },
    0x8480: {
        "Bedeutung": "Gruppe 35: Linearmotor",
        "Anzeige": "35-x",
    },
    0x8600: {
        "Bedeutung": "Gruppe 42: Positionierung",
        "Anzeige": "42-x",
    },
    0x8611: {
        "Bedeutung": "Gruppe 17: Überschreitung Grenzwert Schleppfehler",
        "Anzeige": "17-x",
    },
    0x8611: {
        "Bedeutung": "Gruppe 27: Schleppfehlerüberwachung",
        "Anzeige": "27-x",
    },
    0x8612: {
        "Bedeutung": "Gruppe 40: SW-Endschalter",
        "Anzeige": "40-x",
    },
    0x8680: {
        "Bedeutung": "Positionierung: Fehlende Anschlusspositionierung: Stopp",
        "Anzeige": "42-0",
    },
    0x8681: {
        "Bedeutung": "Positionierung: Drehrichtungsumkehr nicht erlaubt: Stopp",
        "Anzeige": "42-1",
    },
    0x8682: {
        "Bedeutung": "Positionierung: Drehrichtungsumkehr nach Halt nicht erlaubt",
        "Anzeige": "42-2",
    },
    0x8700: {
        "Bedeutung": "Gruppe 34: Feldbus",
        "Anzeige": "34-x",
    },
    0x8780: {
        "Bedeutung": "Keine Synchronisation über Feldbus",
        "Anzeige": "34-0",
    },
    0x8781: {
        "Bedeutung": "Synchronisationsfehler Feldbus",
        "Anzeige": "34-1",
    },
    0x8A00: {
        "Bedeutung": "Gruppe 11: Referenzfahrt",
        "Anzeige": "11-x",
    },
    0x8A00: {
        "Bedeutung": "Gruppe 33: Schleppfehler Encoderemulation",
        "Anzeige": "33-x",
    },
    0x8A80: {
        "Bedeutung": "Fehler beim Start der Referenzfahrt",
        "Anzeige": "11-0",
    },
    0x8A81: {
        "Bedeutung": "Fehler während der Referenzfahrt",
        "Anzeige": "11-1",
    },
    0x8A82: {
        "Bedeutung": "Referenzfahrt: Kein gültiger Nullimpuls",
        "Anzeige": "11-2",
    },
    0x8A83: {
        "Bedeutung": "Referenzfahrt: Zeitüberschreitung",
        "Anzeige": "11-3",
    },
    0x8A84: {
        "Bedeutung": "Referenzfahrt: falscher / ungültiger Endschalter",
        "Anzeige": "11-4",
    },
    0x8A85: {
        "Bedeutung": "Referenzfahrt: I²t / Schleppfehler",
        "Anzeige": "11-5",
    },
    0x8A86: {
        "Bedeutung": "Referenzfahrt: Ende der Suchstrecke",
        "Anzeige": "11-6",
    },
    0x8A87: {
        "Bedeutung": "Schleppfehler Encoderemulation",
        "Anzeige": "33-0",
    },
    0xF000: {
        "Bedeutung": "Gruppe 80: IRQ_0_3",
        "Anzeige": "80-x",
    },
    0xF080: {
        "Bedeutung": "Überlauf Stromregler IRQ",
        "Anzeige": "80-0",
    },
    0xF081: {
        "Bedeutung": "Überlauf Drehzahlregler IRQ",
        "Anzeige": "80-1",
    },
    0xF082: {
        "Bedeutung": "Überlauf Lageregler IRQ",
        "Anzeige": "80-2",
    },
    0xF083: {
        "Bedeutung": "Überlauf Interpolator IRQ",
        "Anzeige": "80-3",
    },
    0xF084: {
        "Bedeutung": "Überlauf Low-Level IRQ",
        "Anzeige": "81-4",
    },
    0xF085: {
        "Bedeutung": "Überlauf MDC IRQ",
        "Anzeige": "81-5",
    },
    0xFF00: {
        "Bedeutung": "Gruppe 28: Betriebsstundenzähler",
        "Anzeige": "28-x",
    },
    0xFF01: {
        "Bedeutung": "Betriebsstundenzähler fehlt",
        "Anzeige": "28-0",
    },
    0xFF02: {
        "Bedeutung": "Betriebsstundenzähler: Schreibfehler",
        "Anzeige": "28-1",
    },
    0xFF03: {
        "Bedeutung": "Betriebsstundenzähler korrigiert",
        "Anzeige": "28-2",
    },
    0xFF04: {
        "Bedeutung": "Betriebsstundenzähler konvertiert",
        "Anzeige": "28-3",
    },
}
