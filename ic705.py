'''
Icom IC-705 Split Controller
Autor: Pascal Pfau (DH1PV)
Benötigte Bibliotheken:
- pip install pyserial
Einstellungen im TRX:
- CI-V USB Echo Back muss 'off' sein
'''

import time
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import threading
import configparser
from pathlib import Path
from tkinter import simpledialog


version = '1.1.0'
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
        self.transverter = {'up':1_968_000_000, 'down':10_345_000_000} # Default bei QO-100
        self.msg_header = bytes([0xfe, 0xfe, self.trx_Adresse, self.controller_Adresse, 0xfd])
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
            'split_on':b'\x0f\x01' # Split on
            }
        self.split = False # Status Split

    def connect(self):
        '''Aufbau der Verbindung zum TRX über Serielle Schnittstelle'''
        with self.lock:
            try:
                self.ic705 = serial.Serial(port=self.serial_port, baudrate=self.baud_rate, timeout=self.time_out, write_timeout=self.write_timeout)
                '''überprüfung ob Verbindung korrekt hergestellt worden ist und ob Gerät auch korrekt Antwortet'''
                self.ic705.write(self._message('vfo_rx'))
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

    def _message(self, key, bcd:bytes=None):
        '''Erstellen der CI-V Message'''
        msg = bytearray(self.msg_header)
        cmd = self.msg_cmd[key]
        if bcd:
            cmd += bcd
        msg[4:4] = cmd
        return bytes(msg)

    def bcd_abfrage(self, key:list):
        '''Abfrage Binär Codierte Dezimalzahl'''
        with self.lock:
            try:
                self.ic705.reset_input_buffer() # Löschen des Empfangspuffers
                for k in key:
                    msg = self._message(k)
                    self.ic705.write(msg) # Abfrage
                    time.sleep(0.01)
                time.sleep(0.1)
                data = self.ic705.read(self.ic705.in_waiting) # Lesen des unformatierten Datenstromes
                return data, None
            except Exception as e:
                return None, e

    def write(self, key, bcd=None):
        '''schreiben von Kommandos'''
        msg = b''
        if self.connected:
            with self.lock:
                try:
                    self.ic705.reset_input_buffer()
                    for k in key:
                        msg += self._message(k, bcd)
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
        freq = f'{freq:010d}' # Auffüllen mit Nullen
        freq_bytes = [int(freq[i]+freq[i+1], 16) for i in range(0, len(freq)-1, 2)] # Bildung BCD-Bytes: 2 Dezimalziffern = 1 Byte
        freq_bytes.reverse() # BCD-Bytes, niederwertige Dezimalziffern zuerst
        return bytes(freq_bytes)

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
            bcd, err_bcd = self.bcd_abfrage(key=['vfo_rx', 'vfo_tx'])
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
        bcd, err_bcd = self.bcd_abfrage(['mode_rx', 'mode_tx'])
        mode, err_mode = self.bcd_to_mode(bcd)
        if (mode[0] != mode[1]):
            self.write(['mode_tx'], mode[0])
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
            except Exception as e:
                print(f'FEHLER in is_split(): {e}')
                return None

    def set_split(self, on_off):
        key = ['split_on'] if on_off else ['split_off']      
        self.write(key)

    def is_tx_enable(self):
        '''Abfrage TX True. Comming soon'''
        pass


class CIV_SettingsManager:
    def __init__(self):
        self.configfile = 'config.ini'
        self.fenster = None
        self.bg_mittel = None

    def einstellungen(self):
        pass

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

    def config_einlesen(self):
        '''Einlesen der config.ini'''
        config = configparser.ConfigParser()
        if Path(self.configfile).is_file():
            config.read(self.configfile)
            ausgabe = {
                'serial_port':config.get('TRANSCEIVER', 'last_com_port') or None,
                'baud_rate':config.getint('TRANSCEIVER', 'baud_rate'),
                'trx_adresse':int(config.get('TRANSCEIVER', 'trx_adresse'), 16),
                'offset':config.getint('OFFSET', 'last_offset'),
                'step':config.getint('OFFSET', 'last_step'),
                'split':config.getboolean('TRANSCEIVER', 'split'),
                'queries_p_sec':config.getint('TRANSCEIVER', 'abfragen_pro_sekunde'),
                'xy':config.get('ALLGEMEIN', 'fensterposition', fallback=None) or None,
                'save_win_pos':config.getboolean('ALLGEMEIN', 'fensterposition_speichern', fallback=False) or False,
                'on_top':config.getboolean('ALLGEMEIN', 'on_top', fallback=False) or False,
                'up':config.getint('OFFSET', 'transverter_up'),
                'down':config.getint('OFFSET', 'transverter_down')
                }
            return ausgabe
        return {}

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


class CIV_GUI:
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
            self.control.transverter['up'] = config['up']
            self.control.transverter['down'] = config['down']
        
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
        self.queries_p_sec = config.get('queries_p_sec', 4)
        self.save_win_pos.set(config.get('save_win_pos', False))
        self.always_on_top.set(config.get('on_top', False))
        self.fenster.attributes('-topmost', config.get('on_top', False)) # Festlegen ob Fenster immer im Vordergrund ist
        if config:
            if config['xy'] is not None:
                self.fenster.geometry(f'+{config["xy"]}')

    def _close(self):
        '''Schließen / Beenden des Programms'''
        if self.start_ft:
            self.stop_frequenz_update_thread()
        x = self.fenster.winfo_x()
        y = self.fenster.winfo_y()
        allgemein = {
            'fensterposition_speichern':self.save_win_pos.get(),
            'fensterposition':f'{x}+{y}' if self.save_win_pos.get() else '',
            'on_top':self.always_on_top.get()
            }
        transceiver = {
            'last_com_port':self.control.serial_port or '',
            'baud_rate':self.control.baud_rate,
            'trx_adresse':f'{self.control.trx_Adresse:02x}', # Speichern als zweistellige HEX-Zahl
            'split':self.checkbu_Split_bool.get(),
            'abfragen_pro_sekunde':self.queries_p_sec
            }
        offset = {
            'last_offset':self.worker.offset,
            'last_step':self.worker.step,
            'transverter_up':self.control.transverter['up'],
            'transverter_down':self.control.transverter['down']
            }
        self.fenster.destroy()
        self.sm.config_schreiben(allgemein, transceiver, offset)

    def _menu(self):        
        '''Menüleiste'''
        self.always_on_top = tk.BooleanVar(value=False)
        self.save_win_pos = tk.BooleanVar()
        self.transverter = tk.BooleanVar()

        self.mLeiste = tk.Menu(self.fenster)
        self.mDatei = tk.Menu(self.mLeiste, tearoff=0)
        self.mDatei.add_command(label='Einstellungen')
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
        self.mTRX.add_checkbutton(label='Split on / off', variable=self.checkbu_Split_bool)
        self.mTRX.add_command(label='TX-Leistung einstellen')

        self.mHilfe = tk.Menu(self.mLeiste, tearoff=0)
        self.mHilfe.add_command(label='Hilfe', command=self.sm.hilfe)
        self.mHilfe.add_separator()
        self.mHilfe.add_command(label='Info')

        self.mLeiste.add_cascade(label='Datei', menu=self.mDatei)
        self.mLeiste.add_cascade(label='Optionen', menu=self.mOptionen)
        self.mLeiste.add_cascade(label='TRX', menu=self.mTRX)
        self.mLeiste.add_cascade(label='Hilfe', menu=self.mHilfe)
        self.fenster['menu'] = self.mLeiste

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
        # self.etOffset.bind('<FocusOut>',self._etOffset_commit)

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

    def _tracking_off(self):
        '''Deaktiviert das automatische Nachstellen des nicht aktiven VFO (Tracking)'''
        self.worker.freq_tracking = False
        self.mTRX.entryconfig(1, label='Tracking Start', command=self._tracking_on)
        self.buTracking.config(background="#0000ff", text='Tracking Start', command=self._tracking_on)

    def _update_lbRXTX_Anzeige(self, freqrx, freqtx, refresh=False):
        '''Anzeige der Frequenzen'''
        if self.start_ft and self.control.connected:
            if self.transverter.get() and self.control.split: # Transvertermode nur bei Split
                mhzrx = (self.control.transverter['down'] + freqrx) / 1_000_000
                mhztx = (self.control.transverter['up'] + freqtx) / 1_000_000
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
                self.control.split = False if self.checkbu_Split_bool.get() else True
                self.worker.set_split(False if self.control.split else True)
                self.buVerbinden.config(text='Trennen', command=self.stop_frequenz_update_thread, background='#ff0000')
                self.mTRX.entryconfig(0, label='Trennen', command=self.stop_frequenz_update_thread)
                self.mTRX.entryconfig(1, state='active')
                self.status_indikator.itemconfig(self.status_indikator_oval, fill='#00ff00')
                self.buTracking.config(state='normal')
                self.cbPorts.config(state='disabled')
                self.buPorts_refresh.config(state='disabled')
                self.ft.start() # Start des Threads

    def stop_frequenz_update_thread(self):
        '''Stoppen der kontinuierlichen Frequenzabfrage'''
        if self.worker.freq_tracking:
            self._tracking_off()
        self.start_ft = False
        self.lbTX_Anzeige_text.set(value='off') # TX-Anzeige auf 'off' schalten
        '''Einstellen der Bedienelemente'''
        self.cbPorts.config(state='readonly')
        self.buPorts_refresh.config(state='normal')
        self.buTracking.config(state='disabled')
        self.control.disconnect()
        self.mTRX.entryconfig(0, label='Verbinden', command=self.start_frequenz_update_thread)
        self.mTRX.entryconfig(1, state='disabled')
        self.buVerbinden.config(text='Verbinden', command=self.start_frequenz_update_thread, background='#00ff00')
        self.status_indikator.itemconfig(self.status_indikator_oval, fill='#ff0000')
        self.lbRX_Anzeige_text.set(value='off') # Anzeige auf "off" stellen

    def frequenz_update_thread(self):
        '''Updateschleife für Anzeige und Tracking'''
        freqrx_alt = 0
        s_old = int(time.time())
        while self.start_ft and self.control.connected:
            s_new = int(time.time())
            delta_s = s_new - s_old
            if  delta_s >= 1 and self.control.split:
                self.worker.mode_switch() # Check ob beide VFO gleichn Mode haben
                s_old = int(time.time())
            self.control.split = self.worker.is_split()
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
                for i in range(100//self.queries_p_sec):
                    '''Warteschleife'''
                    if not self.start_ft or not self.control.connected: # Überprüfung auf Abbruch beim warten
                        break
                    time.sleep(0.01)



if __name__ == "__main__":
    gui=CIV_GUI()
    gui.fenster.mainloop()
