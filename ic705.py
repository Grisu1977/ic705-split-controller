'''
Icom IC-705 Split Controller
Autor: Pascal Pfau (DH1PV)
Benötigte Bibliotheken:
- pip install pyserial pystray pillow
Einstellungen im TRX:
- CI-V USB Echo Back muss 'off' sein
'''

import time # Zeitfunktionen
import serial # PySerial für die serielle Kommunikation
import serial.tools.list_ports # Für die Abfrage der aktiven Ports
import tkinter as tk # GUI mit Tkinter
from tkinter import ttk # Erweiterte Tkinter Widgets
from tkinter import messagebox # Für Messageboxen
from tkinter.simpledialog import Dialog # Für Einstellungsdialoge
import threading # Für Threading
import configparser # Für das Einlesen und Schreiben der INI-Datei
from pathlib import Path # Für den Pfad zur INI-Datei
from PIL import Image # Für Tray-Icon
import pystray as tray # Für das Tray-Icon
from pystray import MenuItem # Für das Tray-Menü
from pystray import Menu # Für das Tray-Menü
import sys, os # Für Ressourcenpfade



version = '1.2.0'
name = 'Icom IC-705 Split Controller'


class CIV_Control:    
    def __init__(self):
        '''
        Initialisiert den Controller für den IC-705.
        Setzt Standardwerte für Adressen, Frequenzen und Offset.
        '''
        self.connected = False
        self.lock = threading.Lock()
        self.serial_port = None # Serielle schnittstelle (Default)
        self.baud_rate = 19200 # Default TRX-Baudrate
        self.time_out = 0.1 # Timeout beim Connect zum TRX via Pyserial
        self.write_timeout = 5
        self.trx_Adresse = 0xa4 # Default TRX-Adresse
        self.controller_Adresse = 0xe0 # Muss in der Regel nicht angepasst werden
        self.offset = 287_500_000 # Default Offset (QO-100)
        self.transverter_offset = {'up':1_968_000_000, 'down':10_345_000_000} # Default bei QO-100
        self.split = False # Status Split
        self.tx_max = 5 # Maximale TX-Leistung in Prozent (Default 5%)
        self.queries_p_sec = 4 # Anzahl Abfragen pro Sekunde ()
        self.msg_cmd = {
            'tx_set':b'\x25\x01', # Setzen Frequenz im TRX (Nicht aktiver VFO)
            'vfo_rx':b'\x25\x00', # Abfrage Frequenz (Aktiver VFO)
            'vfo_tx':b'\x25\x01', # Abfrage Frequenz (Nicht aktiver VFO)
            'tx':b'\x1c\x00', # is TX / RX 
            'xfc':b'\x1c\x02', # is XTC
            'mode_rx':b'\x26\x00', # Mode + Filter (usb, lsb, cw, ...)
            'mode_tx':b'\x26\x01', # Mode + Filter (usb, lsb, cw, ...)
            'is_split':b'\x0f', # Split on / off ?
            'split_off':b'\x0f\x00', # Split off
            'split_on':b'\x0f\x01', # Split on
            'pwr':b'\x14\x0a' # Setzen der TX-Leistung
            }

    def connect(self):
        '''Aufbau der Verbindung zum TRX über Serielle Schnittstelle'''
        with self.lock:
            try:
                self.ic705 = serial.Serial(port=self.serial_port, baudrate=self.baud_rate, timeout=self.time_out, write_timeout=self.write_timeout)
                '''überprüfung ob Verbindung korrekt hergestellt worden ist und ob Gerät auch korrekt Antwortet'''
                self.ic705.write(self.message('vfo_rx'))
                dataTest = self.ic705.read_until()
                if len(dataTest) < 7:
                    self.ic705.close()
                    raise IOError('Keine korrekte Antwort vom Transceiver.\
                                  \rIst der Transceiver eingeschaltet?\
                                  \rIst der Richtige Port gewählt?')
                self.connected = True
                return True, None
            except serial.SerialTimeoutException as e:
                self.ic705 = None
                return False, e
            except serial.PortNotOpenError as e:
                self.ic705 = None
                if self.serial_port is None:
                    e = f'Wähle einen Port\n{e}'
                return False, e
            except Exception as e:
                self.ic705 = None
                return False, e

    def disconnect(self):
        '''Schließen der seriellen Schnittstelle'''
        with self.lock:
            try:
                self.ic705.close()
                self.connected = False
            except Exception as e:
                print(f'Fehler beim Trennen: {e}') # Kontrollausgabe

    def message(self, key, bcd:bytes=None):
        '''Erstellen der CI-V Message'''
        msg_header = bytes([0xfe, 0xfe, self.trx_Adresse, self.controller_Adresse, 0xfd])
        msg = bytearray(msg_header)
        cmd = self.msg_cmd[key]
        if bcd:
            cmd += bcd
        msg[4:4] = cmd
        return bytes(msg)

    def bcd_abfrage(self, keys:list[str]):
        '''
        Abfrage Binär Codierte Dezimalzahl\n
        erlaubte Werte: self.msg_cmd.keys()
        'tx_set'
        'vfo_rx'
        'vfo_tx'
        'tx'
        'xfc'
        'mode_rx'
        'mode_tx'
        'is_split'
        'split_off'
        'split_on'
        'pwr'
        '''
        with self.lock:
            try:
                self.ic705.reset_input_buffer() # Löschen des Empfangspuffers
                for k in keys:
                    msg = self.message(k)
                    self.ic705.write(msg) # Abfrage
                    time.sleep(0.01)
                time.sleep(0.1)
                data = self.ic705.read(self.ic705.in_waiting) # Lesen des unformatierten Datenstromes
                return data, None
            except Exception as e:
                return None, e

    def write(self, keys:list[str], bcd=None):
        '''
        Abfrage Binär Codierte Dezimalzahl\n
        erlaubte Werte: self.msg_cmd.keys()
        'tx_set'
        'vfo_rx'
        'vfo_tx'
        'tx'
        'xfc'
        'mode_rx'
        'mode_tx'
        'is_split'
        'split_off'
        'split_on'
        'pwr'
        '''
        msg = b''
        if self.connected:
            with self.lock:
                try:
                    self.ic705.reset_input_buffer()
                    for k in keys:
                        msg += self.message(k, bcd)
                    self.ic705.write(msg)
                    ok_ng = self.ic705.read_until(b'\xfd')
                    print(f'###   ***   {ok_ng}   ***   ###') # Kontrollausgabe
                    if ok_ng[len(ok_ng)-2] == 0xfa:
                        raise Exception('Fehler 0xFA beim schreiben')
                except serial.PortNotOpenError as e:
                    print(f'Fehler: {e}') # Kontrollausgabe
                except Exception as e:
                    print(f'Fehler beim Schreiben: {e}') # Kontrollausgabe

    def abfrage_Ports(self):
        '''Abfrage "aller" aktiven Ports im System'''
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports[0].startswith('COM'):
            ports = sorted(ports, key=lambda x: int(x.replace('COM', ''))) # Sortieren der Com-Ports
        return ports


class CIV_Worker:
    def __init__(self, bcd_abfrage, write):
        self.bcd_abfrage = bcd_abfrage
        self.write = write
        self.controller_Adresse = None
        self.trx_Adresse = None
        self.offset = None
        self.step = None
        self.freq_tracking = False

    def bcd_to_freq(self, bcd:bytes):
            '''Parsen der 5 Frequenz-Bytes'''
            find_rx = bytes([0xfe,0xfe,self.controller_Adresse,self.trx_Adresse,0x25,0x00])
            find_tx = bytes([0xfe,0xfe,self.controller_Adresse,self.trx_Adresse,0x25,0x01])
            freq_rx = None
            freq_tx = None
            try:
                f_rx = bcd.find(find_rx)
                f_tx = bcd.find(find_tx)
                if bcd.find(bytes([0xfe,0xfe,self.controller_Adresse,self.trx_Adresse,0xfa,0xfd])) != -1: # Fehlerhafte Antwort abfangen
                    raise serial.SerialException('No response from TRX (power off or CI-V inactive)')
                if f_rx != -1 and bcd[f_rx + 11] == 0xfd: # Parsing rx Bytes
                    freq_rx = bcd[f_rx+10:f_rx+5:-1]
                    freq_rx = freq_rx.hex()
                if f_tx != -1 and bcd[f_tx + 11] == 0xfd: # Parsing tx Bytes
                    freq_tx = bcd[f_tx+10:f_tx+5:-1]
                    freq_tx = freq_tx.hex()
                if freq_rx and freq_tx:
                    return int(freq_rx), int(freq_tx), None # Ausgabe der Frequenz in Hz als ganze Zahl
                else:
                    return None, None, None
            except Exception as e:
                return None, None, e

    def freq_to_bcd(self, freq):
        '''Konvertiert Frequenz in BCD-Format (5 Bytes für 10 Ziffern)'''
        t = 1_000_000_000 # Teiler für die höchste Stelle
        bcd = bytearray(5) # 5 Bytes für 10 Ziffern
        for i in range(4, -1, -1): # Schleife für jedes Byte
            hi = (freq // t) # Höhere 4 Bits
            lo = (freq // (t // 10)) # Niedrigere 4 Bits
            bcd[i] = hi << 4 | lo % 10 # BCD-Kodierung
            freq = freq % (t // 10) # Restfrequenz
            t = t // 100 # Teiler für die nächsten 2 Ziffern
        return bytes(bcd)

    def bcd_to_mode(self, bcd:bytes):
        '''Parsing der Mode-Bytes'''
        find_mode_rx = bytes([0xfe, 0xfe, self.controller_Adresse, self.trx_Adresse, 0x26, 0x00])
        find_mode_tx = bytes([0xfe, 0xfe, self.controller_Adresse, self.trx_Adresse, 0x26, 0x01])
        mode_vfo_rx = None
        mode_vfo_tx = None
        try:
            f_m_rx = bcd.find(find_mode_rx)
            f_m_tx = bcd.find(find_mode_tx)
            if bcd.find(bytes([0xfe, 0xfe, self.controller_Adresse, self.trx_Adresse, 0xfa, 0xfd])) != -1: # Fehlerhafte Antwort abfangen
                raise serial.SerialException('No response from TRX (power off or CI-V inactive)')
            if f_m_rx != -1 and bcd[f_m_rx+9] == 0xfd: # Mode rx
                mode_vfo_rx = bcd[f_m_rx+6:f_m_rx+9]
            if f_m_tx != -1 and bcd[f_m_tx+9] == 0xfd: # Mode tx
                mode_vfo_tx = bcd[f_m_tx+6:f_m_tx+9]
            return (mode_vfo_rx, mode_vfo_tx), None # return Mode-Bytes
        except Exception as e:
            return None, e

    def txfreq_set(self, freqtx):
        '''Setzen der Sendefrequenz (Nicht Aktiver VFO) im Funkgerät'''
        bcd = self.freq_to_bcd(freqtx)
        self.write(['tx_set'], bcd)

    def calc_txfreq(self, rx, tx, rx_alt):
        '''Berechnung der TX-Frequenz'''
        diff = None if rx_alt is None else abs(rx - rx_alt)
        change_sign = False
        if tx is not None:
            if rx > tx and self.offset > 0:
                change_sign = True
            elif rx < tx and self.offset < 0:
                change_sign = True
        if diff is not None and (not self.freq_tracking or diff == 0 or diff >= 100_000):
            return None, change_sign
        tx = rx + (-self.offset if (change_sign and tx) else self.offset)
        if 30_000 <= tx < 200_000_000 or 400_000_000 <= tx <= 470_000_000:
            return tx, change_sign
        return None, change_sign

    def freq_update(self):
        '''Abfrage der Frequenzen von "vfo a(b)" und "vfo b(a)"'''
        freq_rx = None
        err_freq = None
        err_bcd = None
        while freq_rx is None and err_bcd is None and err_freq is None:
            bcd, err_bcd = self.bcd_abfrage(keys=['vfo_rx', 'vfo_tx'])
            if bcd is not None:
                print(f'bcd_read: {bcd.hex(' ')}') # Kontrollausgabe
            if err_bcd is None:
                freq_rx, freq_tx, err_freq = self.bcd_to_freq(bcd)
                print(f'freq: {freq_rx} *** {freq_tx}') # Kontrollausgabe
                if err_freq is None and freq_rx:
                    return (freq_rx, freq_tx), None
        msg = 'Die Verbindung wird getrennt. Überprüfe den Transceiver\n'
        if err_bcd:
            msg += f'\nBCD Fehler (read): {err_bcd}'
        if err_freq:
            msg += f'\nFrequenz Fehler: {err_freq}'
        return None, msg

    def mode_switch(self):
        '''Abfrage und Synchronisation der Modes zwischen VFO A(B) und VFO B(A)'''
        bcd, err_bcd = self.bcd_abfrage(['mode_rx', 'mode_tx']) # Abfrage der Modes
        if bcd is not None:
            mode, err_mode = self.bcd_to_mode(bcd)
        if mode is not None and err_mode is None and (mode[0] != mode[1]): # Vergleich der Modes
            self.write(['mode_tx'], mode[0]) # Setzen des Modes im TX VFO
        if err_mode or err_bcd:
            msg = ''
            if err_bcd:
                msg += f'\nMode lese Fehler (bcd): {err_bcd}'
            if err_mode:
                msg += f'\nMode Parsing-Fehler: {err_mode}'
            print(msg)

    def is_split(self):
        '''Abfrage ob am TRX Split on / off ist'''
        find_split_on = bytes([0xfe, 0xfe, self.controller_Adresse, self.trx_Adresse, 0x0f, 0x01, 0xfd])
        find_split_off = bytes([0xfe, 0xfe, self.controller_Adresse, self.trx_Adresse, 0x0f, 0x00, 0xfd])
        for i in range(5):
            bcd, err_bcd = self.bcd_abfrage(['is_split'])
            try:
                if err_bcd:
                    raise err_bcd
                s_on = bcd.find(find_split_on)
                s_off = bcd.find(find_split_off)
                if bcd.find(bytes([0xfe, 0xfe, self.controller_Adresse, self.trx_Adresse, 0xfa, 0xfd])) != -1: # Fehlerhafte Antwort abfangen
                    raise serial.SerialException('No response from TRX (power off or CI-V inactive)')
                if s_on != -1:
                    return True
                if s_off != -1:
                    return False
                if i == 4:
                        raise Exception('Unklarer Fehler in is_split')
            except serial.SerialTimeoutException as e:
                print(f'Timeoutfehler in is_split: {e}')
                return None
            except Exception as e:
                print(f'FEHLER in is_split(): {e}')
                return None

    def set_split(self, on_off):
        '''Setzen von Split on / off'''
        key = ['split_on'] if on_off else ['split_off']
        self.write(key)

    def set_tx_pwr(self, pwr:int=5):
        '''Setzen der TX-Leistung (0 bis 100 %)'''
        value = round(pwr * 255 / 100) # Umrechnung in 0-255
        byte1 = value // 100 # Hunderterstelle
        rest = (value % 100) # Zehner- und Einerstelle
        byte2 = (rest // 10) << 4 | rest % 10 # BCD Kodierung
        bcd = bytes([byte1, byte2]) # BCD-Bytes
        self.write(['pwr'], bcd)

    def is_tx_enable(self):
        '''Abfrage TX True. Comming soon'''
        pass


class CIV_SettingsManager:
    '''Verwaltung der Einstellungen und Dialoge'''
    def __init__(self):
        self.configfile = 'config.ini'
        self.fenster = None
        self.bg_mittel = None
        self.transverteron_at_start = False
        self.ontop_at_start = False
        self.dialog_set = {}

    def config_einlesen(self):
        '''Einlesen der config.ini'''
        config = configparser.ConfigParser()
        if Path(self.configfile).is_file():
            config.read(self.configfile)
            ausgabe = {
                'save_win_pos':config.getboolean('ALLGEMEIN', 'fensterposition_speichern', fallback=False) or False,
                'xy':config.get('ALLGEMEIN', 'fensterposition', fallback=None) or None,
                'on_top':config.getboolean('ALLGEMEIN', 'on_top', fallback=False) or False,
                'minimize_to_tray':config.getboolean('ALLGEMEIN', 'minimize_to_tray', fallback=False) or False,
                'serial_port':config.get('TRANSCEIVER', 'last_com_port') or None,
                'baud_rate':config.getint('TRANSCEIVER', 'baud_rate'),
                'trx_adresse':int(config.get('TRANSCEIVER', 'trx_adresse'), 16),
                'split':config.getboolean('TRANSCEIVER', 'split'),
                'queries_p_sec':config.getint('TRANSCEIVER', 'abfragen_pro_sekunde'),
                'tx_max':config.getint('TRANSCEIVER', 'tx_max'),
                'offset':config.getint('OFFSET', 'last_offset'),
                'step':config.getint('OFFSET', 'last_step'),
                'up':config.getint('OFFSET', 'transverter_up'),
                'down':config.getint('OFFSET', 'transverter_down'),
                'transverter_on_start':config.getboolean('OFFSET', 'transverter_on_start')
                }
            return ausgabe
        return {}

    def config_schreiben(self, allgemein, transceiver, offset):
        '''Parsen und Schreiben der config.ini'''
        config = configparser.ConfigParser()
        config['ALLGEMEIN'] = allgemein
        config['TRANSCEIVER'] = transceiver
        config['OFFSET'] = offset
        with open(self.configfile, 'w', encoding='utf-8') as cf: # öffnen und schreiben der *.ini
            cf.write( # Kommentarblock in der Datei
                '; #==================================================#\n' \
                '; | Konfigurationsdatei für IC-705 Split Controller  |\n' \
                '; | Autor Pascal Pfau (DH1PV)                        |\n' \
                '; | Diese Datei wird automatisch erzeugt.            |\n' \
                '; | Manuelle Änderungen sind möglich,                |\n' \
                '; | geschehen auf eigene Gefahr. Sollte das Programm |\n' \
                '; | nach Änderungen nicht mehr wie erwartet laufen,  |\n' \
                '; | kann diese Datei gefahrlos gelöscht werden.      |\n' \
                '; | Die Nutzung des Programms geschieht auf eigene   |\n' \
                '; | Gefahr.                                          |\n' \
                '; #==================================================#\n\n'
            )
            config.write(cf)

    def hilfe(self):
        '''Generierung eines Hilfe- und Infofensters'''
        info = f'''\
{name} v{version}
Autor: Pascal Pfau (DH1PV)
eMail: dh1pv@darc.de
©2026\n
Funktionen:
- Frequenznachführung für Crossbandbetrieb
- Offset-Korrektur
- Feinabstimmung der TX-Frequenz
- Automatische Mode-Synchronisation zwischen VFO A(B) und VFO B(A)
- Unterstützung Splitsteuerung on / off
- Anzeige der Tatsächlichen TX-Frequenz
- Speicherung der Einstellungen\n
Release Notes:
02-01-2026 v1.0.0
- Erste stabile Version (1.0.0)
- Verbesserte Code-Struktur und Stabilität
- Einführung einer Worker-Klasse
- Bugfixes
18-01-2026 v1.1.0
- Unterstützung des Split-Betriebs
- Automatische Mode-Synchronisation zwischen VFO A => VFO B
- Synchronisationsinterval über config.ini jetzt anpassbar
- Aktualisierung der Hilfe im Programm
- Bugfixes und Verbesserung der Stabilität
- CIV_Control.write() jetzt flexibler

Disclaimer:
Die Benutzung des Programms geschieht auf eigene Gefahr. Für Schäden an Geräten (Computer, TRX, etc.) \
und Software, sowie Datenverlusten, übernehme ich keinerlei Haftung.\n
Erste schritte:
Nach dem Start als erstes den richtigen Seriellen Port auswählen. Meist in der Geräteverwaltung mit \
CI-V gekennzeichnet. Danach auf Verbinden klicken. Jetzt wird die Aktuelle RX-Frequenz angezeigt. Zum \
Nachführen der TX-Frequenz Tracking Start klicken. Nun wird auch die Aktuelle TX-Frequenz angezeigt und \
automatisch anhand des aktuell eingestelltem Offsets gesetzt.
Das Offset kann manuell in Herz eingegeben werden oder aber in Einzelschritten, einstellbar unter \
Schrittweite, mit "+" und "-" werden.
Wenn der haken bei Slit gesetzt ist schaltet sich split mit ein, wenn nicht markiert ist \
Split im TRX off. Split kann auch direkt im TRX gesetzt werden. Eine rückmeldung erfolgt über die Checkbox.
In der config.ini kann eingestellt werden wie oft Pro sec eine Abfrage der Frequenz vom TRX gemacht wird. \
Standard ist 4. Mehr als 10 bei Verbindungen via Bluetooth und 25 via USB sind hier nicht sinnvoll und \
können zu Fehlern führen.\n
! ! ! ACHTUNG ! ! !
Damit das Programm richtig funktioniert ist es erforderlich das sowohl 
MENU >> Connectors >> CI-V >> CI-V USB Echo Back = OFF
und bei Nutzung via Bluetooth
MENU >> Bluetooth Set >> Data Device Set >> Serialport Funktion = CI-V (Echo Back OFF)
eingestellt ist.\
'''
        '''Bau des Infofensters'''
        fensterHilfe = tk.Toplevel(self.fenster, background=self.bg_mittel)
        fensterHilfe.title('Info / Hilfe')
        fensterHilfe.resizable(1,1)
        fensterHilfe.transient(self.fenster)
        fensterHilfe.grab_set()
        fensterHilfe.rowconfigure(0, weight=1)
        fensterHilfe.columnconfigure(0, weight=1)

        frText = ttk.Frame(fensterHilfe)
        frText.columnconfigure(0, weight=1)
        frText.rowconfigure(0, weight=1)
        frText.grid(row=0, column=0,  padx=5, pady=5, sticky='nsew')
        sbText = tk.Scrollbar(frText, orient='vertical')
        sbText.grid(row=0, column=1, sticky='ns')

        text = tk.Text(frText, wrap='word', width=75, background=self.bg_mittel, foreground="#00ff00", yscrollcommand=sbText.set, font=(None,14))
        text.rowconfigure(0, weight=1)
        text.columnconfigure(0, weight=1)
        text.grid(row=0, column=0, sticky='nsew')
        text.insert(1.0, info)
        text.config(state='disabled')
        sbText.config(command=text.yview)

        buOK = ttk.Button(fensterHilfe, text='Schließen', command=fensterHilfe.destroy)
        buOK.grid(row=1, column=0, pady=(0, 5))
        
        fensterHilfe.update_idletasks()
        screen_w = fensterHilfe.winfo_screenwidth()
        screen_h = fensterHilfe.winfo_screenheight()
        width = fensterHilfe.winfo_width()
        height = fensterHilfe.winfo_height()
        x = ((screen_w // 2) - (width // 2)) * 0.33
        y = ((screen_h // 2) - (height // 2)) * 0.33
        fensterHilfe.geometry(f'{width}x{height}+{round(x)}+{round(y)}')

    def tx_pwr(self, tx_max):
        '''Generierung eines Eingabedialogs für die TX-Leistung'''
        class TXPowerDialog(Dialog):
            def __init__(self, parent, title=None, tx_max:int=5):
                self.tx_max = tx_max
                self.result = None
                super().__init__(parent, title)

            def body(self, master):
                self.resizable(False, False)
                tk.Label(master, text=f'TX-Leistung Einstellen').grid(column=0, row=0, columnspan=2)
                tk.Label(master, text=f'0 bis max {self.tx_max} % :').grid(column=0, row=1)
                self.tx_entry = tk.Entry(master, width=7)
                self.tx_entry.grid(column=1, row=1)
                self.tx_entry.insert(0, '')
                return self.tx_entry

            def validate(self):
                try:
                    tx_value = int(self.tx_entry.get())
                    if not (0 <= tx_value <= self.tx_max):
                        raise ValueError
                    return True
                except ValueError:
                    messagebox.showerror('Fehler', f'Die TX-Leistung muss eine Zahl zwischen 0 und {self.tx_max} sein.')
                    return False

            def apply(self):
                self.result = int(self.tx_entry.get())

        self.dialog_tx = TXPowerDialog(parent=self.fenster, title='TX-Leistung einstellen', tx_max=tx_max).result

    def open_settings(self):
        '''Generierung eines Einstellungsdialogs'''
        class SettingDialog(Dialog):
            def __init__(self, parent, title=None, setting_values:dict=None):
                self.setting_values = setting_values
                self.result = None
                super().__init__(parent, title)

            def body(self, master):
                '''Erstellen des Dialogfensters'''
                self.resizable(False, False)
                frame1 = tk.Frame(master)
                frame1.columnconfigure(0, weight=1)
                frame1.grid(column=0, row=0, sticky='ns')
                frame2 = tk.Frame(master)
                frame2.columnconfigure(0, weight=1)
                frame2.grid(column=1, row=0, sticky='ns')
                frame3 = tk.Frame(master)
                frame3.columnconfigure(0, weight=1)
                frame3.grid(column=2, row=0, sticky='ns')

                tk.Label(frame1, text='Transceiver').grid(column=0, row=0, columnspan=2 ) 
                tk.Label(frame1, text='TRX-Adresse:').grid(column=0, row=1)
                tk.Label(frame1, text='Baudrate:').grid(column=0, row=2)
                tk.Label(frame1, text='Abfragen p. sec:').grid(column=0, row=3)
                tk.Label(frame1, text='Max TX PWR (%):').grid(column=0, row=4)
                self.trx_adr = tk.Entry(frame1, width=7)
                self.trx_adr.grid(column=1, row=1)
                self.trx_adr.insert(0, self.setting_values.get('trx_adr', ''))
                self.br = tk.Entry(frame1, width=7)
                self.br.grid(column=1, row=2)
                self.br.insert(0, self.setting_values.get('br', ''))
                self.qps = tk.Entry(frame1, width=7)
                self.qps.grid(column=1, row=3)
                self.qps.insert(0, self.setting_values.get('abs', ''))
                self.tx_max = tk.Entry(frame1, width=7)
                self.tx_max.grid(column=1, row=4)
                self.tx_max.insert(0, self.setting_values.get('tx_max', ''))

                tk.Label(frame2, text='Transverter').grid(column=0, row=0, columnspan=2)
                tk.Label(frame2, text='Immer ein\nbeim Start').grid(column=0, row=1)
                tk.Label(frame2, text='Down:').grid(column=0, row=2)
                tk.Label(frame2, text='Up:').grid(column=0, row=3)
                self.transv_on_bool = tk.BooleanVar()
                self.transverter_on = tk.Checkbutton(frame2, variable=self.transv_on_bool)
                self.transverter_on.grid(column=1, row=1)
                self.transv_on_bool.set(self.setting_values.get('t_on'))
                self.td = tk.Entry(frame2, width=12)
                self.td.grid(column=1, row=2)
                self.td.insert(0, self.setting_values.get('down', ''))
                self.tu = tk.Entry(frame2, width=12)
                self.tu.grid(column=1, row=3)
                self.tu.insert(0, self.setting_values.get('up', ''))

                tk.Label(frame3, text='Allgemein').grid(column=0, row=0, columnspan=2, sticky='n')
                tk.Label(frame3, text='Ins Tray minimieren').grid(column=0, row=1)
                tk.Label(frame3, text='Beim Start im Vordergrund').grid(column=0, row=2)
                self.minimize_to_tray = tk.BooleanVar()
                self.minimize_to_tray_check = tk.Checkbutton(frame3, variable=self.minimize_to_tray)
                self.minimize_to_tray_check.grid(column=1, row=1)
                self.minimize_to_tray.set(self.setting_values.get('min_to_tray', False))
                self.always_on_top = tk.BooleanVar()
                self.always_on_top_check = tk.Checkbutton(frame3, variable=self.always_on_top)
                self.always_on_top_check.grid(column=1, row=2)
                self.always_on_top.set(self.setting_values.get('on_top', False))
                return super().body(master)

            def validate(self):
                '''Validierung der Eingabewerte'''
                try:
                    self.trx_adresse = int(self.trx_adr.get(), 16)
                except ValueError:
                    messagebox.showerror('Fehler', 'TRX-Adresse muss eine Hexadezimale Zahl sein.')
                    return False
                if not (0x00 <= self.trx_adresse <= 0xFF):
                    messagebox.showerror('Fehler', 'TRX-Adresse muss zwischen 00 und FF liegen.')
                    return False
                try:
                    self.baudrate = int(self.br.get())
                except ValueError:
                    messagebox.showerror('Fehler', 'Baudrate muss eine Zahl sein.')
                    return False
                if not self.baudrate > 0:
                    messagebox.showerror('Fehler', 'Baudrate muss größer als 0 sein.')
                    return False
                try:
                    self.queries_p_sec = int(self.qps.get())
                except ValueError:
                    messagebox.showerror('Fehler', 'Abfrage pro Sekunde muss eine Zahl sein')
                    return False
                if not (0 < self.queries_p_sec <= 25):
                    messagebox.showerror('Fehler', 'Abfrage pro Sekunde muss twischen\n 1 und 25 sein')
                    return False
                try:
                    self.tx_max_pwr = int(self.tx_max.get())
                except ValueError:
                    messagebox.showerror('Fehler', 'Max TX PWR muss eine Zahl sein')
                    return False
                if not (0 < self.queries_p_sec <= 100):
                    messagebox.showerror('Fehler', 'Max TX PWR muss twischen\n 1 und 100 sein')
                    return False
                try:
                    self.transv_down = int(self.td.get())
                    self.transv_up = int(self.tu.get())
                except ValueError:
                    messagebox.showerror('Fehler', 'Transverter-Offset muss eine Zahl sein.')
                    return False
                return True

            def apply(self):
                '''Speichern der Eingabewerte'''
                self.result = {
                    'trx_adr':self.trx_adresse,
                    'br':self.baudrate,
                    'abs':self.queries_p_sec,
                    't_on':self.transv_on_bool.get(),
                    'down':self.transv_down,
                    'up':self.transv_up,
                    'min_to_tray':self.minimize_to_tray.get(),
                    'on_top':self.always_on_top.get(),
                    'tx_max':self.tx_max_pwr
                    }

        self.dialog_set = SettingDialog(parent=self.fenster, title='Einstellungen', setting_values=self.dialog_set).result

class CIV_GUI_TRAY:
    '''Verwaltung des Tray Icons'''
    def __init__(self):
        '''Initialisierung des Tray Icons'''
        self.icon = {'green':Image.open(self.resource_path('green.png')),'red':Image.open(self.resource_path('red.png')),'yellow_green':Image.open(self.resource_path('yellow_green.png'))}
        self.tray_icon = tray.Icon(name='TrayIcon', icon=self.icon['red'], title=name, menu=self.build_tray_menu())
        self.tray_icon.run_detached()
        self.minimizetotray = False

    def resource_path(self, relative_path:str):
        '''Ermitteln des Pfades für Ressourcen'''
        try:
            base_path = sys._MEIPASS
        except Exception:
            base_path = os.path.abspath('.') # Aktueller Pfad
        return os.path.join(base_path, relative_path) # Absoluter Pfad zur Ressource

    def build_tray_menu(self, icon:str=None):
        if icon is not None:
            self.tray_icon.icon = self.icon[icon]
        if not self.start_ft:
            return Menu(MenuItem(text='Fenster öffnen', action=self.show_window),
                        MenuItem(text='Verbinden', action=self.start_frequenz_update_thread),
                        MenuItem(text='Tracking Start', action=self._tracking_on, enabled=False),
                        MenuItem(text='Beenden', action=self.tray_close))
        elif self.start_ft and not self.worker.freq_tracking:
            return Menu(MenuItem(text='Fenster öffnen', action=self.show_window),
                        MenuItem(text='Trennen', action=self.stop_frequenz_update_thread),
                        MenuItem(text='Tracking Start', action=self._tracking_on, enabled=True),
                        MenuItem(text='Beenden', action=self.tray_close))
        else:
            return Menu(MenuItem(text='Fenster öffnen', action=self.show_window),
                        MenuItem(text='Trennen', action=self.stop_frequenz_update_thread),
                        MenuItem(text='Tracking Stop', action=self._tracking_off, enabled=True),
                        MenuItem(text='Beenden', action=self.tray_close))

    def tray_close(self):
        self.fenster.after(0, self._close)

    def show_window(self):
        self.fenster.deiconify()

    def minimize_to_tray(self, event=None):
        self.fenster.withdraw()


class CIV_GUI(CIV_GUI_TRAY):
    def __init__(self):
        '''
        Initialisiert die GUI für den IC-705 Controller.
        Erstellt alle UI-Elemente und startet den Controller.
        '''
        self.fenster = tk.Tk()
        self.start_ft = False
        self.fenster.title(f'{name} - v{version}') # Name Titelleiste Fenster / Programmname
        self.fenster.resizable(False, False) # Größe des Fensters wird durch seine Inhalte bestimmt
        self.fenster.protocol('WM_DELETE_WINDOW', self._close)
        self.sm = CIV_SettingsManager()
        config = self.sm.config_einlesen()

        '''Control'''
        self.control = CIV_Control()
        if config:
            self.control.serial_port = config['serial_port']
            self.control.baud_rate = config['baud_rate']
            self.control.trx_Adresse = config['trx_adresse']
            self.control.split =config['split']
            self.control.transverter_offset['up'] = config['up']
            self.control.transverter_offset['down'] = config['down']
            self.control.queries_p_sec = config['queries_p_sec']
            self.control.tx_max = config['tx_max']
        
        '''Worker'''
        self.worker = CIV_Worker(bcd_abfrage=self.control.bcd_abfrage, write=self.control.write)
        self.worker.step = config.get('step', 10)
        self.worker.offset = config.get('offset', self.control.offset)
        self.worker.trx_Adresse = self.control.trx_Adresse
        self.worker.controller_Adresse = self.control.controller_Adresse

        '''GUI'''
        self._setup_user_interface()
        self._menu()
        self.sm.fenster = self.fenster
        self.sm.bg_mittel = self.bg_mittel
        self.save_win_pos.set(config.get('save_win_pos', False))
        self.always_on_top.set(config.get('on_top', self.sm.ontop_at_start))
        self.sm.ontop_at_start = self.always_on_top.get()
        self.fenster.attributes('-topmost', self.always_on_top.get()) # Festlegen ob Fenster immer im Vordergrund ist
        if config:
            if config['xy'] is not None:
                self.fenster.geometry(f'+{config["xy"]}')
            self.transverter.set(config.get('transverter_on_start', self.transverter))
        self.sm.transverteron_at_start = self.transverter.get()

        '''Tray Icon'''
        super().__init__()
        self.minimizetotray = config.get('minimize_to_tray', self.minimizetotray)
        if self.minimizetotray:
            self.fenster.bind('<Unmap>', self.minimize_to_tray)

    def _close(self):
        '''Schließen / Beenden des Programms'''
        if self.start_ft:
            self.stop_frequenz_update_thread()
        x = self.fenster.winfo_x()
        y = self.fenster.winfo_y()
        allgemein = {
            'fensterposition_speichern':self.save_win_pos.get(),
            'fensterposition':f'{x}+{y}' if self.save_win_pos.get() else '',
            'on_top':self.sm.ontop_at_start,
            'minimize_to_tray':self.minimizetotray
            }
        transceiver = {
            'last_com_port':self.control.serial_port or '',
            'baud_rate':self.control.baud_rate,
            'trx_adresse':f'{self.control.trx_Adresse:02x}', # Speichern als zweistellige HEX-Zahl
            'split':self.checkbu_Split_bool.get(),
            'abfragen_pro_sekunde':self.control.queries_p_sec,
            'tx_max': self.control.tx_max
            }
        offset = {
            'last_offset':self.worker.offset,
            'last_step':self.worker.step,
            'transverter_up':self.control.transverter_offset['up'],
            'transverter_down':self.control.transverter_offset['down'],
            'transverter_on_start':self.sm.transverteron_at_start
            }
        self.tray_icon.stop()
        self.fenster.destroy()
        self.sm.config_schreiben(allgemein, transceiver, offset)

    def _menu(self):        
        '''Menüleiste'''
        self.always_on_top = tk.BooleanVar(value=False)
        self.save_win_pos = tk.BooleanVar()
        self.transverter = tk.BooleanVar()

        self.mLeiste = tk.Menu(self.fenster)
        self.mDatei = tk.Menu(self.mLeiste, tearoff=0)
        self.mDatei.add_command(label='Einstellungen', command=self._change_config)
        self.mDatei.add_separator()
        self.mDatei.add_command(label='Beenden', command=self._close)

        self.mOptionen = tk.Menu(self.mLeiste, tearoff=0)
        self.mOptionen.add_checkbutton(label='Immer im Vordergrund', variable=self.always_on_top,
                                       command=lambda: self.fenster.attributes('-topmost', self.always_on_top.get()))
        self.mOptionen.add_checkbutton(label='Fensterposition Speichern', variable=self.save_win_pos)
        self.mOptionen.add_checkbutton(label='Transvertermodus', variable=self.transverter)

        self.mTRX = tk.Menu(self.mLeiste, tearoff=0)
        self.mTRX.add_command(label='Verbinden', command=self.start_frequenz_update_thread)
        self.mTRX.add_command(label='Tracking Start', command=self._tracking_on, state='disabled')
        self.mTRX.add_checkbutton(label='Split on / off', variable=self.checkbu_Split_bool, command=lambda:self.worker.set_split(self.checkbu_Split_bool.get()))
        self.mTRX.add_command(label='TX-Leistung einstellen', command=self._tx_pwr_set, state='disabled')

        self.mHilfe = tk.Menu(self.mLeiste, tearoff=0)
        self.mHilfe.add_command(label='Hilfe', command=self.sm.hilfe)
        self.mHilfe.add_separator()
        self.mHilfe.add_command(label='Info')

        self.mLeiste.add_cascade(label='Datei', menu=self.mDatei)
        self.mLeiste.add_cascade(label='Optionen', menu=self.mOptionen)
        self.mLeiste.add_cascade(label='TRX', menu=self.mTRX)
        self.mLeiste.add_cascade(label='Hilfe', menu=self.mHilfe)
        self.fenster['menu'] = self.mLeiste

    def _tx_pwr_set(self):
        '''Dialog zum Setzen der TX-Leistung'''
        self.sm.tx_pwr(self.control.tx_max)
        if self.sm.dialog_tx is not None:
            self.worker.set_tx_pwr(self.sm.dialog_tx)

    def _change_config(self):
        '''Aktualisierung der Konfiguration'''
        self.sm.dialog_set.clear()
        self.sm.dialog_set = {
            'trx_adr':f'{self.control.trx_Adresse:02x}',
            'br':f'{self.control.baud_rate}',
            'abs':f'{self.control.queries_p_sec}',
            't_on':self.sm.transverteron_at_start,
            'down':f'{self.control.transverter_offset['down']}',
            'up':f'{self.control.transverter_offset['up']}',
            'tx_max':f'{self.control.tx_max}',
            'min_to_tray':self.minimizetotray,
            'on_top':self.sm.ontop_at_start
        }
        self.sm.open_settings()
        if self.sm.dialog_set is not None:
            self.worker.trx_Adresse = self.sm.dialog_set.get('trx_adr')
            self.control.trx_Adresse = self.worker.trx_Adresse
            self.control.baud_rate = self.sm.dialog_set.get('br')
            self.control.queries_p_sec = self.sm.dialog_set.get('abs')
            self.sm.transverteron_at_start = self.sm.dialog_set.get('t_on')
            self.control.transverter_offset['down'] = self.sm.dialog_set.get('down')
            self.control.transverter_offset['up'] = self.sm.dialog_set.get('up')
            self.control.tx_max = self.sm.dialog_set.get('tx_max')
            self.minimizetotray = self.sm.dialog_set.get('min_to_tray', self.minimizetotray)
            self.fenster.bind('<Unmap>', self.minimize_to_tray) if self.minimizetotray else self.fenster.unbind('<Unmap>')
            self.sm.ontop_at_start = self.sm.dialog_set.get('on_top', self.sm.ontop_at_start)

    def _setup_user_interface(self):
        '''Definition der Farben im Fenster'''
        self.bg_ausgabe = "#000000" # Hintergrund Frequenzanzeige RX / TX
        self.fg_ausgabeRX = "#00ff00" # Schriftfarbe RX
        self.fg_ausgabeTX = "#ff0000" # Schriftfarbe TX
        self.bg_dunkel = "#1e293b" # Hintergrund des Fensters
        self.bg_mittel = "#334155" # Hintergrund der Frameinhalte
        self.rahmen_hell = "#dbdbbd" # Farbe vom Rahmen des Frame-Widget
        self.schrift = "#ffffff" # Farbe der Schrift
        self.schrift_dis = "#aaaaaa" # Farbe der Schrift

        '''Theme Konfiguration für ttk'''
        style = ttk.Style()
        style.theme_use('clam')
        style.map('My.TCheckbutton',
                  background=[('active', self.bg_mittel),('disabled', self.bg_mittel),
                              ('selected', self.bg_mittel), ('!selected', self.bg_mittel)],
                  foreground=[('active',self.schrift), ('disabled', self.schrift_dis),
                              ('selected', self.schrift), ('!selected', self.schrift)]
                              )

        '''Bau der Benutzeroberfläche'''
        self.fenster.config(background=self.bg_dunkel) # Setzen der Hintergrundfarbe

        '''Titelframe'''
        self.frTitel = tk.Frame(self.fenster,
                                background=self.bg_mittel,
                                highlightbackground=self.rahmen_hell,
                                highlightthickness=1,
                                width=425,
                                height=50)
        self.frTitel.grid(row=0, column=0, padx=5, pady=5, sticky='ew')
        self.frTitel.grid_propagate(False)
        self.frTitel.columnconfigure(0,weight=0)
        self.frTitel.columnconfigure(1,weight=1)
        self.frTitel.rowconfigure(0, weight=1)
        self.lbTitel = tk.Label(self.frTitel,
                                background=self.bg_mittel,
                                text=name, # Name des Programms
                                font=(None, 16, 'bold'),
                                foreground=self.schrift)
        self.lbTitel.grid(row=0, column=0)
        self.frTitel_re = tk.Frame(self.frTitel, background=self.bg_mittel)
        self.frTitel_re.grid(row=0, column=1, sticky='e')
        self.status_indikator = tk.Canvas(self.frTitel_re,
                                          width=15,
                                          height=15,
                                          bg=self.bg_mittel,
                                          highlightthickness=0)
        self.status_indikator.grid(row=0, column=0)
        self.status_indikator_oval = self.status_indikator.create_oval(0, 0, 15, 15, fill='#ff0000')
        self.buHilfe = tk.Button(self.frTitel_re,
                                 command=self.sm.hilfe,
                                 text='?', 
                                 font=(None, 13, 'bold'),
                                 foreground='red',
                                 background=self.bg_mittel,
                                 borderwidth=0,
                                 highlightthickness=0,
                                 activebackground=self.bg_mittel)
        self.buHilfe.grid(row=0, column=1)

        '''Steuerelemente Verbinden und Tracking'''
        self.frVerbinden = tk.Frame(self.fenster, background=self.bg_dunkel)
        self.frVerbinden.grid(row=1, column=0, padx=5, pady=2, sticky='w')
        self.lbPorts = tk.Label(self.frVerbinden,
                               relief='flat',
                               background=self.bg_dunkel,
                               text='COM Port:',
                               foreground=self.schrift)
        self.lbPorts.grid(row=0, column=0)

        self.cbPorts_var = tk.StringVar()
        self.cbPorts = ttk.Combobox(self.frVerbinden,
                                    width=12,
                                    values=self.control.abfrage_Ports(),
                                    state='readonly',
                                    textvariable=self.cbPorts_var)
        self.cbPorts.grid(row=0, column=1, padx=5, pady=0)
        self.cbPorts_var.trace_add('write', self._cbPorts_auswahl)
        self.cbPorts_var.set(str(self.control.serial_port))

        self.buPorts_refresh = tk.Button(self.frVerbinden,
                                         command=self._refresh_ports,
                                         text=chr(0x27F3))
        self.buPorts_refresh.grid(row=0, column=2, padx=5, pady=0)
        self.buVerbinden = tk.Button(self.frVerbinden,
                                     command=self.start_frequenz_update_thread,
                                     text='Verbinden',
                                     background='#00ff00',
                                     font=(None,10,'bold'),
                                     width=10)
        self.buVerbinden.grid(row=0, column=3, padx=5, pady=0)
        self.buTracking = tk.Button(self.frVerbinden,
                                    text='Tracking Start',
                                    background="#0000FF",
                                    font=(None,10,'bold'),
                                    state='disabled',
                                    command=self._tracking_on)
        self.buTracking.grid(row=0, column=4, padx=5, pady=0)

        '''Anzeigeelemente für RX und TX'''
        self.frAnzeige = tk.Frame(self.fenster,
                                  background=self.bg_mittel,
                                  highlightbackground=self.rahmen_hell,
                                  highlightthickness=1)
        self.frAnzeige.grid(row=2, column=0, padx=5, pady=5, sticky='ew')
        self.frAnzeige.rowconfigure(0, weight=1)
        self.frAnzeige.rowconfigure(1, weight=1)
        self.frAnzeige.columnconfigure(0, weight=1, uniform='frAnzeige')
        self.frAnzeige.columnconfigure(1, weight=1, uniform='frAnzeige')
        self.lbRX = tk.Label(self.frAnzeige,
                             text='RX Frequenz',
                             font=(None, 12, 'bold'),
                             background=self.bg_mittel,
                             foreground=self.fg_ausgabeRX)
        self.lbRX.grid(row=0, column=0, padx=2, pady=0)
        self.lbTX = tk.Label(self.frAnzeige,
                             text='TX Frequenz',
                             font=(None, 12, 'bold'),
                             background=self.bg_mittel,
                             foreground=self.fg_ausgabeTX)
        self.lbTX.grid(row=0, column=1, padx=2, pady=0)
        self.lbRX_Anzeige_text = tk.StringVar(value='off')
        self.lbRX_Anzeige = tk.Label(self.frAnzeige,
                                     relief='sunken',
                                     background=self.bg_ausgabe,
                                     foreground=self.fg_ausgabeRX,
                                     textvariable=self.lbRX_Anzeige_text,
                                     font=('Courier New', 15),
                                     width=15,
                                     height=1)
        self.lbRX_Anzeige.grid(row=1, column=0, padx=2, pady=2, sticky='ew')
        self.lbTX_Anzeige_text = tk.StringVar(value='off')
        self.lbTX_Anzeige = tk.Label(self.frAnzeige,
                                     relief='sunken',
                                     background=self.bg_ausgabe,
                                     foreground=self.fg_ausgabeTX,
                                     textvariable=self.lbTX_Anzeige_text,
                                     font=('Courier New', 15),
                                     width=15,
                                     height=1)
        self.lbTX_Anzeige.grid(row=1, column=1, padx=2, pady=2, sticky='ew')

        '''Einstellelemente für Offset'''
        self.frOffset = tk.Frame(self.fenster,
                                 background=self.bg_mittel,
                                 highlightbackground=self.rahmen_hell,
                                 highlightthickness=1)
        self.frOffset.grid(row=3, column=0, padx=5, pady=5, sticky='ew')
        self.frOffset.columnconfigure(0, weight=1, uniform='frOffset')
        self.frOffset.columnconfigure(1, weight=1, uniform='frOffset')
        self.lbOffset = tk.Label(self.frOffset,
                                 background=self.bg_mittel,
                                 text='Frequenz - Offset (Hz)',
                                 font=(None, 8),
                                 foreground=self.schrift)
        self.lbOffset.grid(row=0, column=0, sticky='w')
        self.lbOffset_schritt = tk.Label(self.frOffset,
                                         background=self.bg_mittel,
                                         text='Schrittweite',
                                         font=(None, 8),
                                         foreground=self.schrift)
        self.lbOffset_schritt.grid(row=0, column=1, sticky='w')
        self.frOffset_uli = tk.Frame(self.frOffset, background=self.bg_mittel)
        self.frOffset_uli.grid(row=1, column=0, padx=2, pady=0, sticky='nsew')
        self.frOffset_uli.columnconfigure(1,weight=1)
        self.frOffset_uli.columnconfigure(2,weight=1)
        self.etOffset_var = tk.StringVar(value=self.worker.offset)
        self.etOffset = tk.Entry(self.frOffset_uli,
                                 font=('Courier New', 12),
                                 width=12,
                                 validate='key',
                                 validatecommand=(self.frOffset_uli.register(self._etOffset_filter), '%P'),
                                 textvariable=self.etOffset_var)
        self.etOffset.grid(row=0, column=0, padx=5, pady=5)
        self.etOffset.bind('<Return>',self._etOffset_commit)

        self.buOffset_plus = tk.Button(self.frOffset_uli,
                                       command=lambda:self._etOffset_pm(1),
                                       text='+')
        self.buOffset_plus.grid(row=0, column=2, padx=1, pady=5, sticky='ew')
        self.buOffset_minus = tk.Button(self.frOffset_uli,
                                        command=lambda:self._etOffset_pm(-1),
                                        text='-')
        self.buOffset_minus.grid(row=0, column=1, padx=1, pady=5, sticky='ew')

        self.frOffset_ure = tk.Frame(self.frOffset,background=self.bg_mittel)
        self.frOffset_ure.grid(row=1, column=1, padx=2, pady=0, sticky='nsew')
        self.frOffset_ure.rowconfigure(0, weight=1)
        self.frOffset_ure.columnconfigure(1, weight=1)
        self.cbStep_var = tk.StringVar(value=self.worker.step)
        self.cbStep = ttk.Combobox(self.frOffset_ure,
                                      values=[1,10,100,1_000,10_000,100_000,1_000_000],
                                      textvariable=self.cbStep_var,
                                      state='readonly',
                                      width=12)
        self.cbStep.grid(row=0, column=0, sticky='w')
        self.cbStep.bind('<<ComboboxSelected>>', self._cbStep_auswahl)

        self.checkbu_Split_bool = tk.BooleanVar()
        self.checkbu_Split_bool.set(self.control.split)
        self.checkbu_Split = ttk.Checkbutton(self.frOffset_ure,
                                            style='My.TCheckbutton',
                                            state='active',
                                            text='Split (on / off)', 
                                            variable=self.checkbu_Split_bool,
                                            command=lambda:self.worker.set_split(self.checkbu_Split_bool.get()))
        self.checkbu_Split.grid(row=0, column=1)

    def _etOffset_filter(self, value):
        '''Filtern der Benutzereingaben'''
        return value == '' or value == '-' or value.lstrip("-").isdigit()

    def _etOffset_commit(self, event=None): # "event" wird von "bind" immer mitgeliefert, aber hier nicht verwendet
        '''Einstellen des Benutzeroffset'''
        raw_etOffset = self.etOffset_var.get()
        try:
            value_etOffst = int(raw_etOffset)
            if -1_000_000_000 < value_etOffst < 1_000_000_000:
                self.worker.offset=value_etOffst
                self.refresh_lbRXTX_Anzeige() # Aktualisierung der Anzeige
            else:
                raise ValueError
        except ValueError as e:
            self.etOffset.focus_set()
            self.etOffset.select_range(0, tk.END)
            messagebox.showerror(title='Fehler', message='Bitte eine gültige Frequenz in Herz eingeben\nNur Ziffern 0 ... 9')

    def _etOffset_pm(self, plusminus):
        '''Feinjustierung des Offsets'''
        var = int(self.etOffset_var.get())
        step = int(self.worker.step)
        self.etOffset_var.set(var + step * plusminus)
        self.worker.offset=int(self.etOffset_var.get()) # einstellen der Richtung Plus / Minus
        self.refresh_lbRXTX_Anzeige() # Aktualisierung der Anzeige

    def _etOffset_sign_change(self):
        '''Ändern des  (Offset)'''
        self.worker.offset *= -1
        self.etOffset_var.set(self.worker.offset)

    def _cbPorts_auswahl(self, *args):
        '''Setzen eines Ports'''
        self.control.serial_port = None if self.cbPorts_var.get() == 'None' else self.cbPorts_var.get()

    def _cbStep_auswahl(self, *args):
        '''Auswahl der Offset Schrittweite'''
        self.worker.step = self.cbStep.get()

    def _refresh_ports(self):
        '''Aktualisierung der Ports'''
        ports = self.control.abfrage_Ports()
        self.cbPorts.config(values=ports)

    def _tracking_on(self):
        '''Aktiviert das automatische Nachstellen des nicht aktiven VFO (Tracking)'''
        self.worker.freq_tracking = True
        self.mTRX.entryconfig(1, label='Tracking Stop', command=self._tracking_off)
        self.buTracking.config(background="#ffdd00", text='Tracking Stop', command=self._tracking_off)
        self.refresh_lbRXTX_Anzeige() # Aktualisierung der Anzeige
        self.tray_icon.menu = self.build_tray_menu('yellow_green')
        self.tray_icon.update_menu()

    def _tracking_off(self):
        '''Deaktiviert das automatische Nachstellen des nicht aktiven VFO (Tracking)'''
        self.worker.freq_tracking = False
        self.mTRX.entryconfig(1, label='Tracking Start', command=self._tracking_on)
        self.buTracking.config(background="#0000ff", text='Tracking Start', command=self._tracking_on)
        self.tray_icon.menu = self.build_tray_menu('green')
        self.tray_icon.update_menu()

    def _update_lbRXTX_Anzeige(self, freqrx, freqtx, refresh=False):
        '''Anzeige der Frequenzen'''
        if self.start_ft and self.control.connected:
            if self.transverter.get() and self.control.split: # Transvertermode nur bei Split
                mhzrx = (self.control.transverter_offset['down'] + freqrx) / 1_000_000
                mhztx = (self.control.transverter_offset['up'] + freqtx) / 1_000_000
            else:
                mhzrx = freqrx / 1_000_000
                mhztx = freqtx / 1_000_000 if self.control.split else mhzrx # Entscheidung welche TX-VFO-Frequenz angezeigt wird. Aktiv oder Sub
            self.lbRX_Anzeige_text.set(value=f'{mhzrx:.6f} MHz')
            if self.worker.freq_tracking and freqtx is not None:
                self.lbTX_Anzeige_text.set(value=f'{mhztx:.6f} MHz')
                if refresh and self.control.split:
                    self.worker.txfreq_set(freqtx)
                return
            self.lbTX_Anzeige_text.set(value=f'{mhztx:.6f} MHz')

    def refresh_lbRXTX_Anzeige(self):
        '''Aktuallisiert die TX-Anzeige'''
        if self.start_ft and self.control.connected:
            freq, err = self.worker.freq_update()
            if freq is None and err is not None:
                messagebox.showerror(title='Fehler', message=err)
                return
            freqtx, change_sign = self.worker.calc_txfreq(rx=freq[0], tx=0, rx_alt=None)
            if freqtx is None:
                freqtx = freq[1]
            self._update_lbRXTX_Anzeige(freqrx=freq[0], freqtx=freqtx, refresh=True)

    def start_frequenz_update_thread(self):
        '''Starten der kontinuierlichen Frequenzabfrage'''
        if not self.start_ft:
            connect, err = self.control.connect()
            if not connect:
                messagebox.showerror(title='Verbindungsfehler', message=err)
            else:
                self.ft = threading.Thread(target=self.frequenz_update_thread, daemon=True) 
                self.start_ft = True
                '''Einstellen der Bedienelemente'''
                self.status_indikator.itemconfig(self.status_indikator_oval, fill='#00ff00')
                self.control.split = False if self.checkbu_Split_bool.get() else True
                self.worker.set_split(False if self.control.split else True)
                self.buPorts_refresh.config(state='disabled')
                self.buVerbinden.config(text='Trennen', command=self.stop_frequenz_update_thread, background='#ff0000')
                self.buTracking.config(state='normal')
                self.mDatei.entryconfig(0, state='disabled')
                self.mTRX.entryconfig(0, label='Trennen', command=self.stop_frequenz_update_thread)
                self.mTRX.entryconfig(1, state='active')
                self.mTRX.entryconfig(3, state='active')
                self.cbPorts.config(state='disabled')
                self.tray_icon.menu = self.build_tray_menu('green')
                self.tray_icon.update_menu()
                self.ft.start() # Start des Threads

    def stop_frequenz_update_thread(self):
        '''Stoppen der kontinuierlichen Frequenzabfrage'''
        if self.worker.freq_tracking:
            self._tracking_off()
        self.start_ft = False
        self.lbTX_Anzeige_text.set(value='off') # TX-Anzeige auf 'off' schalten
        '''Einstellen der Bedienelemente'''
        self.status_indikator.itemconfig(self.status_indikator_oval, fill='#ff0000')
        self.cbPorts.config(state='readonly')
        self.buPorts_refresh.config(state='normal')
        self.buVerbinden.config(text='Verbinden', command=self.start_frequenz_update_thread, background='#00ff00')
        self.buTracking.config(state='disabled')
        self.control.disconnect()
        self.mDatei.entryconfig(0, state='normal')
        self.mTRX.entryconfig(0, label='Verbinden', command=self.start_frequenz_update_thread)
        self.mTRX.entryconfig(1, state='disabled')
        self.mTRX.entryconfig(3, state='disabled')
        self.lbRX_Anzeige_text.set(value='off') # Anzeige auf "off" stellen
        self.tray_icon.menu = self.build_tray_menu('red')
        self.tray_icon.update_menu()

    def frequenz_update_thread(self):
        '''Updateschleife für Anzeige und Tracking'''
        freqrx_alt = 0
        s_old = int(time.time())
        while self.start_ft and self.control.connected:
            s_new = int(time.time())
            delta_s = s_new - s_old
            if  self.control.split is not None and delta_s >= 1 and self.control.split:
                self.worker.mode_switch() # Check ob beide VFO gleichn Mode haben
                s_old = int(time.time())
            self.control.split = self.worker.is_split()
            if self.control.split is not None:
                self.fenster.after(0, lambda:self.checkbu_Split_bool.set(self.control.split))
            if self.start_ft and self.control.connected:
                freq, err = self.worker.freq_update()
            if freq is None and err is not None:
                self.fenster.after(0, self.stop_frequenz_update_thread) # Bei Fehler Stopp des UpdateThreads
                self.fenster.after(0, lambda:messagebox.showerror(title='Fehler', message=err))
                break
            else:
                if freq is not None:
                    freqtx, change_sign = self.worker.calc_txfreq(freq[0], freq[1], freqrx_alt)
                    if change_sign:
                        self.fenster.after(0, self._etOffset_sign_change)
                    if freqtx is not None and self.control.split and self.start_ft and self.control.connected:
                        self.worker.txfreq_set(freqtx)
                        self.fenster.after(0, self._update_lbRXTX_Anzeige, freq[0], freqtx)
                    else:
                        self.fenster.after(0, self._update_lbRXTX_Anzeige, freq[0], freq[1])
                    freqrx_alt = freq[0]
                for i in range(100//self.control.queries_p_sec):
                    '''Warteschleife'''
                    if not self.start_ft or not self.control.connected: # Überprüfung auf Abbruch beim warten
                        break
                    time.sleep(0.01)



if __name__ == "__main__":
    gui=CIV_GUI()
    gui.fenster.mainloop()