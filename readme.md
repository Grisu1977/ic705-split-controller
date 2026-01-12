# IC-705 Split Controller

Ein grafisches Python-Tool zur Steuerung und automatischen Nachführung des Split-Betriebs am  
**Icom IC-705** über die **CI-V Schnittstelle**.

Das Programm liest kontinuierlich die RX-Frequenz des aktiven VFO aus und setzt – abhängig vom
eingestellten Offset – automatisch die TX-Frequenz auf dem nicht aktiven VFO.  
Der Fokus liegt auf einfachem, stabilem Betrieb und klarer Bedienung.

> Hinweis: Die README beschreibt den aktuellen Entwicklungsstand (`main`).
> Die letzte veröffentlichte Version ist v1.0.0.


---

## Funktionen

- Anzeige der aktuellen RX- und TX-Frequenz
- Automatische TX-Frequenz-Nachführung (Tracking)
- Frei einstellbarer Frequenz-Offset (positiv / negativ)
- Feinjustierung des Offsets über Schrittweite
- Serielle CI-V Kommunikation (USB / virtuell COM)
- Grafische Oberfläche mit Tkinter
- Speicherung der letzten Einstellungen in einer `config.ini`
- Statusanzeige für Verbindungszustand
- Automatische umschaltung des VFO-Modes von primär nach sekundär bei aktivem Tracking.

---

## Voraussetzungen

- Python 3.x
- Abhängigkeiten:
  pip install pyserial

## Transceiver-Einstellungen (IC-705)

Damit das Programm korrekt funktioniert, müssen folgende Einstellungen gesetzt sein:

- USB / CI-V Betrieb

MENU → Set → Connectors → CI-V → CI-V USB Echo Back = OFF

- Bluetooth (optional)

MENU → Set → Bluetooth Set → Data Device Set → Serialport Funktion = CI-V (Echo Back OFF)

## Nutzung

- Programm starten

- Richtigen COM-Port auswählen (Unter Windows im Gerätemanager meist mit CI-V gekennzeichnet)

- Auf Verbinden klicken
  → RX-Frequenz wird angezeigt

- Tracking Start aktivieren
  → TX-Frequenz wird automatisch anhand des Offsets gesetzt

- Offset bei Bedarf anpassen:
   → Direkte Eingabe (Hz)

- Feinjustierung mit + / – und Schrittweite

## Konfiguration

Die Datei config.ini wird automatisch erzeugt und enthält u. a.:

Letzten verwendeten COM-Port
Baudrate
CI-V Adresse
Offset und Schrittweite

Manuelle Änderungen sind möglich, erfolgen aber auf eigene Verantwortung.
Bei Problemen kann die Datei gefahrlos gelöscht werden.

## Bekannte Einschränkungen

Aktuell nur für Windows getestet
Linux-Unterstützung ist geplant
WLAN-Verbindung noch nicht implementiert

## Geplante Erweiterungen

Linux-Implementierung
Speicherung der Fensterposition
Tray-Icon
Manuelles Setzen von RX/TX-Frequenzen
Optionale automatische Aktivierung von Split
CI-V über WLAN
Erweiterte Offset-Validierung

## Lizenz & Haftungsausschluss

Die Nutzung des Programms erfolgt auf eigene Gefahr.
Für Schäden an Hard- oder Software sowie Datenverluste wird keine Haftung übernommen.

## Autor

Pascal Pfau (DH1PV)
eMail: dh1pv@darc.de
