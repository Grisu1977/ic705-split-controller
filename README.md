# IC-705 Split Controller

Ein grafisches Python-Tool zur Steuerung und automatischen Nachführung des Split-Betriebs am  
**Icom IC-705** über die **CI-V Schnittstelle**.

Das Programm liest kontinuierlich die RX-Frequenz des aktiven VFO aus und setzt – abhängig vom
eingestellten Offset – automatisch die TX-Frequenz auf dem sekundären VFO.
Die Anzeige und Logik orientieren sich dabei konsequent am tatsächlichen Verhalten des Transceivers.

> ### Hinweis:
> Die README beschreibt den aktuellen Entwicklungsstand (`master`).
> Die letzte veröffentlichte Version ist v1.1.0.

---

## Funktionsübersicht

- Anzeige der aktuellen RX- und TX-Frequenz
- Unterstützung des Split-Betriebs
- Split OFF → TX auf aktivem VFO (VFO A)
- Split ON → TX auf sekundärem VFO (VFO B)
- Automatische TX-Frequenz-Nachführung (Tracking)
- Frei einstellbarer Frequenz-Offset (positiv / negativ)
- Feinjustierung des Offsets über wählbare Schrittweite
- Anzeige des aktuellen Split-Status
- Anzeige des Verbindungsstatus zum Transceiver
- Automatische Mode-Synchronisation zwischen VFO A und VFO B
- Serielle CI-V-Kommunikation (USB / virtueller COM-Port (Bluetooth))
- Grafische Benutzeroberfläche mit Tkinter
- Persistente Speicherung der letzten Einstellungen in einer config.ini
- Trayicon mit Menü
- Transvertermode

---

## Voraussetzungen

- Python 3.x
- Abhängigkeiten:
  pip install pyserial pystray pillow

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

- Split kann manuell ein- oder ausgeschaltet werden
  (das Programm erzwingt keinen Split-Betrieb)

## Konfiguration

Die Datei config.ini wird beim Beenden automatisch geschrieben und beim Start wieder eingelesen.
Gespeichert werden unter anderem:

- letzter verwendeter COM-Port
- Baudrate
- CI-V-Adresse
- Offset und Schrittweite

Manuelle Änderungen sind möglich, erfolgen jedoch auf eigene Verantwortung.
Bei Problemen kann die Datei jederzeit gefahrlos gelöscht werden.

## Bekannte Einschränkungen

Aktuell nur für Windows getestet
Linux-Unterstützung bedingt möglich
WLAN-Verbindung noch nicht implementiert

## Geplante Erweiterungen

- Linux-Implementierung
- Speicherung der Fensterposition
- Minimierung ins Tray-Icon 
- Manuelles Setzen von RX/TX-Frequenzen
- CI-V-Kommunikation über WLAN
- Erweiterte Offset-Validierung

## Lizenz & Haftungsausschluss

Die Nutzung des Programms erfolgt auf eigene Gefahr.
Für Schäden an Hard- oder Software sowie Datenverluste wird keine Haftung übernommen.

## Autor

Pascal Pfau (DH1PV)
eMail: dh1pv@darc.de

---
## Mitarbeit

Dieses Projekt wird ausschließlich vom Autor gepflegt.

Pull Requests werden in der Regel nicht angenommen und nur nach vorheriger
Absprache berücksichtigt. Vorschläge, Hinweise oder Fehlermeldungen können
gerne über Issues eingereicht werden.

Es besteht kein Anspruch darauf, dass Beiträge übernommen werden.
---