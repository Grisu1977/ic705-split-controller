'''
Docstring
Icom IC-705 Split Controller
Autor: Pascal Pfau (DH1PV)
Benötigte Bibliotheken:
- pip install pyserial
Einstellungen im TRX:
- CI-V USB Echo Back muss 'off' sein
Zukünftige Versionen / Ideen:
- [✓] Automatisches angleichen des Modes VFO A / VFO B bei wechsel
- [] Einschalten des Splitbetriebs bei Tracking Start (Optional Split Ja / Nein)
- [] Hinzufügen eines Tray-Icons
- [] Optionale Speicherung der Fensterposition
- [] Anzeige TX / RX
- [] Bandwahl TX / RX
- [] Linux Implementierung
- [] Manuelles setzen der TX/RX Frequenz mit Berechnung und setzen der Offsetfrequenz
- [] Verbindung via WLAN
- [] Verbesserung des Filters ob gültiger Offset (Liegt Frequenz im Gültigen Bereich vom TRX)
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


version = '1.0.0'


class CIV_Control:    
    def __init__(self):
        '''
        Initialisiert den Controller für den IC-705.
        Setzt Standardwerte für Adressen, Frequenzen und Offset.
        '''
        self.configfile = 'config.ini'
        self.connected = False
        self.lock = threading.Lock()
        self.serial_port = None # Serielle schnittstelle (Default)
        self.baud_rate = 19200 # Default TRX-Baudrate
        self.time_out = 0.1 # Timeout beim Connect zum TRX via Pyserial
        self.write_timeout = 5
        self.trx_Adresse = 0xa4 # Default TRX-Adresse
        self.controller_Adresse = 0xe0 # Muss in der Regel nicht angepasst werden
        self.offset = 287_500_000 # Default Offset (QO-100)
        self.step = 10 # Default Schrittweite für manuelles nachjustieren
        self.msg_header = bytes([0xfe, 0xfe, self.trx_Adresse, self.controller_Adresse, 0xfd])
        self.msg_cmd = {
            'tx_set':b'\x25\x01', # Setzen Frequenz im TRX (Nicht aktiver VFO)
            'vfo_rx':b'\x25\x00', # Abfrage Frequenz (Aktiver VFO)
            'vfo_tx':b'\x25\x01', # Abfrage Frequenz (Nicht aktiver VFO)
            'tx':b'\x1c\x00', # is TX / RX 
            'xfc':b'\x1c\x02', # is XTC
            'mode_rx':b'\x26\x00', # Mode + Filter (usb, lsb, cw, ...)
            'mode_tx':b'\x26\x01', # Mode + Filter (usb, lsb, cw, ...)
            }

    def config_einlesen(self):
        '''Einlesen der config.ini'''
        config = configparser.ConfigParser()
        if Path(self.configfile).is_file():
            config.read(self.configfile)
            if not config['Transceiver']['Last_COM_Port'] == 'None':
                self.serial_port = config['Transceiver']['Last_COM_Port']
            self.baud_rate = int(config['Transceiver']['Baud_Rate'])
            self.trx_Adresse = int(config['Transceiver']['TRX_Adresse'],16)
            self.offset = int(config['Offset']['Last_Offset'])
            self.step = config['Offset']['Last_Step']

    def config_schreiben(self):
        '''Parsen und Schreiben der config.ini'''
        config = configparser.ConfigParser()
        config['Transceiver'] = {
            'Last_COM_Port':str(self.serial_port),
            'Baud_Rate':self.baud_rate,
            'TRX_Adresse':f'{self.trx_Adresse:02x}' # Speichern als zweistellige HEX-Zahl
            }
        config['Offset'] = {'Last_Offset':self.offset,'Last_Step':self.step}
        with open(self.configfile, 'w') as cf: # öffnen und schreiben der *.ini
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
        with self.lock:
            try:
                msg = self._message(key, bcd)
                self.ic705.reset_input_buffer()
                self.ic705.write(msg)
                ok_ng = self.ic705.read_until(b'\xfd')
                print(f'###   ***   {ok_ng}   ***   ###') # Kontrollausgabe
                if ok_ng[len(ok_ng)-2] == 0xfa:
                    raise Exception('Fehler 0xFA beim schreiben')
            except Exception as e:
                print(f'Fehler beim Frequenz setzen: {e}') # Kontrollausgabe

    def abfrage_Ports(self):
        '''Abfrage "aller" aktiven Ports im System'''
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports[0].startswith('COM'):
            ports = sorted(ports, key=lambda x: int(x.replace('COM', ''))) # Sortieren der Com-Ports
        return ports


class CIV_Worker:
    def __init__(self, bcd_abfrage, write, trx_adr, contr_adr, offset=0, step=0):
        self.write = write
        self.bcd_abfrage = bcd_abfrage
        self.trx_Adresse = trx_adr
        self.controller_Adresse = contr_adr
        self.freq_tracking = False
        self.offset = offset
        self.step = step

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
                if self.freq_tracking and f_tx != -1 and bcd[f_tx + 11] == 0xfd: # Parsing tx Bytes
                    freq_tx = bcd[f_tx+10:f_tx+5:-1]
                    freq_tx = freq_tx.hex()
                if freq_rx and freq_tx:
                    return int(freq_rx), int(freq_tx), None # Ausgabe der Frequenz in Hz als ganze Zahl
                elif freq_rx:
                    return int(freq_rx), None, None # Ausgabe der Frequenz in Hz als ganze Zahl
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
        '''Parsing der ber Mode-Bytes'''
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

    def txfreq_set(self, freqtx, start_ft, connected):
        '''Setzen der Sendefrequenz (Nicht Aktiver VFO) im Funkgerät'''
        if start_ft and connected:
            bcd = self.freq_to_bcd(freqtx)
            self.write('tx_set', bcd)

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

    def freq_update(self, start_ft, connected):
        '''Abfrage der Frequenzen von "vfo a" und "vfo b"'''
        if start_ft and connected:
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
        return None, None

    def mode_switch(self):
        if self.freq_tracking:
            bcd, err_bcd = self.bcd_abfrage(['mode_rx', 'mode_tx'])
            mode, err_mode = self.bcd_to_mode(bcd)
            if (mode[0] != mode[1]):
                self.write('mode_tx', mode[0])
                # return True, None
            if err_mode or err_bcd:
                # msg = 'Die Verbindung wird getrennt. Überprüfe den Transceiver\n'
                if err_bcd:
                    msg += f'\nMode lese Fehler (bcd): {err_bcd}'
                if err_mode:
                    msg += f'\nMode Parsing-Fehler: {err_mode}'
                print(msg)
                input()
                # return False, msg
        # return None, None
    
    def is_tx_enable(self):
        pass


class CIV_GUI:
    def __init__(self):
        '''
        Initialisiert die GUI für den IC-705 Controller.
        Erstellt alle UI-Elemente und startet den Controller.
        '''
        self.fenster = tk.Tk()
        self.fenster.title('IC-705 Split Controller') # Name Titelleiste Fenster / Programmname
        self.fenster.resizable(False, False) # Größe des Fensters wird durch seine Inhalte bestimmt
        self.fenster.protocol('WM_DELETE_WINDOW', self._closeFenster)
        self.control = CIV_Control()
        self.control.config_einlesen()
        self.start_ft = False
        self.worker = CIV_Worker(bcd_abfrage=self.control.bcd_abfrage,
                                 write=self.control.write,
                                 trx_adr=self.control.trx_Adresse,
                                 contr_adr=self.control.controller_Adresse,
                                 offset=self.control.offset,
                                 step=self.control.step)
        self._setup_user_interface()

    def _closeFenster(self):
        '''Schließen / Beenden des Programms'''
        if self.start_ft:
            self.stop_frequenz_update_thread()
        self.control.offset = self.worker.offset
        self.control.step = self.worker.step
        self.fenster.destroy()
        self.control.config_schreiben()

    def _setup_user_interface(self):

        style = ttk.Style()
        style.theme_use('clam')

        '''Definition der Farben im Fenster'''
        self.bg_ausgabe = "#000000" # Hintergrund Frequenzanzeige RX / TX
        self.fg_ausgabeRX = "#00ff00" # Schriftfarbe RX
        self.fg_ausgabeTX = "#ff0000" # Schriftfarbe TX
        self.bg_dunkel = "#1e293b" # Hintergrund des Fensters
        self.bg_mittel = "#334155" # Hintergrund der Frameinhalte
        self.rahmen_hell = "#dbdbbd" # Farbe vom Rahmen des Frame-Widget
        self.schrift = "#ffffff" # Farbe der Schrift

        self.fenster.configure(background=self.bg_dunkel) # Setzen der Hintergrundfarbe

        '''Bau der Benutzeroberfläche'''
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
                                text='Icom IC-705 Split Controller', # Name des Programms
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
                                 command=self.hilfe,
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
        self.frAnzeige.columnconfigure(0, weight=1, uniform='col')
        self.frAnzeige.columnconfigure(1, weight=1, uniform='col')
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
        self.frOffset.columnconfigure(0, weight=1, uniform='col')
        self.frOffset.columnconfigure(1, weight=1, uniform='col')
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
        self.frOffset_uli = tk.Frame(self.frOffset,
                                     background=self.bg_mittel)
        self.frOffset_uli.grid(row=1, column=0, padx=2, pady=0, sticky='w')
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
                                       text='+',
                                       width=2,
                                       height=1)
        self.buOffset_plus.grid(row=0, column=2, padx=1, pady=5)
        self.buOffset_minus = tk.Button(self.frOffset_uli,
                                        command=lambda:self._etOffset_pm(-1),
                                        text='-',
                                        width=2,
                                        height=1)
        self.buOffset_minus.grid(row=0, column=1, padx=1, pady=5)

        self.frOffset_ure = tk.Frame(self.frOffset)
        self.frOffset_ure.grid(row=1, column=1, padx=2, pady=0, sticky='w')
        self.cbStep_var = tk.StringVar(value=self.worker.step)
        self.cbStep = ttk.Combobox(self.frOffset_ure,
                                      values=[1,10,100,1_000,10_000,100_000,1_000_000],
                                      textvariable=self.cbStep_var,
                                      state='readonly',
                                      width=12)
        self.cbStep.grid(row=0, column=0)
        self.cbStep.bind('<<ComboboxSelected>>', self._cbStep_auswahl)

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
        # einstellen der Richtung Plus / Minus
        self.worker.offset=int(self.etOffset_var.get())
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
        self.cbPorts.configure(values=ports)

    def _tracking_on(self):
        '''Aktiviert das automatische Nachstellen des nicht aktiven VFO (Tracking)'''
        self.worker.freq_tracking = True
        self.buTracking.config(background="#ffdd00", text='Tracking Stop', command=self._tracking_off)
        time.sleep(0.1)
        self.refresh_lbRXTX_Anzeige() # Aktualisierung der Anzeige

    def _tracking_off(self):
        '''Deaktiviert das automatische Nachstellen des nicht aktiven VFO (Tracking)'''
        self.worker.freq_tracking = False
        self.buTracking.config(background="#0000ff", text='Tracking Start', command=self._tracking_on)
        self.lbTX_Anzeige_text.set(value='off') # TX-Anzeige auf 'off' schalten

    def _update_lbRXTX_Anzeige(self, freqrx, freqtx=None, refresh=False):
        '''Anzeige der Frequenzen'''
        if self.start_ft and self.control.connected:
            mhzrx = freqrx / 1_000_000
            self.lbRX_Anzeige_text.set(value=f'{mhzrx:.6f} MHz')
            if self.worker.freq_tracking and freqtx is not None:
                mhztx = freqtx / 1_000_000
                self.lbTX_Anzeige_text.set(value=f'{mhztx:.6f} MHz')
                if refresh:
                    self.worker.txfreq_set(freqtx, self.start_ft, self.control.connected)

    def refresh_lbRXTX_Anzeige(self):
        '''Aktuallisiert die TX-Anzeige'''
        if self.start_ft and self.worker.freq_tracking:
            freq, err = self.worker.freq_update(self.start_ft, self.control.connected)
            if freq is None and err is not None:
                messagebox.showerror(title='Fehler', message=err)
                return
            freq = self.worker.calc_txfreq(rx=freq[0], tx=0, rx_alt=None)
            self._update_lbRXTX_Anzeige(freqrx=freq[0], freqtx=freq[0], refresh=True)

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
                self.buVerbinden.config(text='Trennen', command=self.stop_frequenz_update_thread, background='#ff0000')
                self.status_indikator.itemconfig(self.status_indikator_oval, fill='#00ff00')
                self.buTracking.config(state='normal')
                self.cbPorts.config(state='disabled')
                self.buPorts_refresh.config(state='disabled')
                self.ft.start() # Start des Threads

    def stop_frequenz_update_thread(self):
        '''Stoppen der kontinuierlichen Frequenzabfrage'''
        self.start_ft = False
        if self.worker.freq_tracking:
            self._tracking_off()
        self.control.disconnect()
        '''Einstellen der Bedienelemente'''
        self.cbPorts.config(state='readonly')
        self.buPorts_refresh.config(state='normal')
        self.buTracking.config(state='disabled')
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
            if  delta_s >= 1:
                self.worker.mode_switch()
                s_old = int(time.time())
            freq, err = self.worker.freq_update(self.start_ft, self.control.connected)
            if freq is None and err is not None:
                self.fenster.after(0, self.stop_frequenz_update_thread) # Bei Fehler Stopp des UpdateThreads
                self.fenster.after(0, lambda:messagebox.showerror(title='Fehler', message=err))
                break
            else:
                freqtx, change_sign = self.worker.calc_txfreq(freq[0], freq[1], freqrx_alt)
                if change_sign:
                    self.fenster.after(0, self._etOffset_sign_change)
                if freqtx is not None:
                    self.worker.txfreq_set(freqtx, self.start_ft, self.control.connected)
                self.fenster.after(0, self._update_lbRXTX_Anzeige, freq[0], freq[1])
                freqrx_alt = freq[0]
                for i in range(10):
                    '''Warteschleife'''
                    if not self.start_ft or not self.control.connected: # Überprüfung auf Abbruch beim warten
                        break
                    time.sleep(0.01)

    def hilfe(self):
        '''Generierung eines Hilfe- und Infofensters'''
        info = f'''\
Icom IC-705 Split Controller v{version}
Autor: Pascal Pfau (DH1PV)
eMail: dh1pv@darc.de
©2026\n
Funktionen:
- Frequenznachführung für Crossbandbetrieb
- Offset-Korrektur
- Feinabstimmung der TX-Frequenz
- Speicherung der Einstellungen\n
Release Notes:
02-01-2026 v1.0.0
- Erste stabile Version (1.0.0)
- Verbesserte Code-Struktur und Stabilität
- Einführung einer Worker-Klasse
- Bugfixes

Disclaimer:
Die Benutzung des Programms geschieht auf eigene Gefahr. Für Schäden an Geräten (Computer, TRX, etc.) \
und Software, sowie Datenverlusten, übernehme ich keinerlei Haftung.\n
Erste schritte:
Nach dem Start als erstes den richtigen Seriellen Port auswählen. Meist in der Geräteverwaltung mit \
CI-V gekennzeichnet. Danach auf Verbinden klicken. Jetzt wird die Aktuelle RX-Frequenz angezeigt. Zum \
Nachführen der TX-Frequenz Tracking Start klicken. Nun wird auch die Aktuelle TX-Frequenz angezeigt und \
automatisch anhand des aktuell eingestelltem Offsets gesetzt.
Das Offset kann manuell in Herz eingegeben werden oder aber in Einzelschritten, einstellbar unter \
Schrittweite, mit "+" und "-" eingestellt werden.\n
! ! ! ACHTUNG ! ! !
Damit das Programm richtig funktioniert ist es erforderlich das sowohl 
MENU >> Connectors >> CI-V >> CI-V USB Echo Back = OFF
und bei Nutzung via Bluetooth
MENU >> Bluetooth Set >> Data Device Set >> Serialport Funktion = CI-V (Echo Back OFF)
eingestellt ist.\
'''
        '''Bau des Infofensters'''
        fensterHilfe = tk.Toplevel(self.fenster, background=self.bg_dunkel)
        fensterHilfe.title('Info / Hilfe')
        fensterHilfe.resizable(0,0)
        fensterHilfe.transient(self.fenster)
        fensterHilfe.grab_set()
            
        frText = ttk.Frame(fensterHilfe)
        frText.grid(row=0, column=0, padx=5, pady=5)
        sbText = tk.Scrollbar(frText, orient='vertical')
        sbText.grid(row=0, column=1, sticky='ns')

        text = tk.Text(frText, wrap='word', width=55, background=self.bg_mittel, foreground="#00ff00", yscrollcommand=sbText.set)
        text.grid(row=0, column=0)
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
        x = (screen_w // 2) - (width // 2)
        y = ((screen_h // 2) - (height // 2)) * 0.66
        fensterHilfe.geometry(f'{width}x{height}+{x}+{round(y)}')


if __name__ == "__main__":
    gui=CIV_GUI()
    gui.fenster.mainloop()
