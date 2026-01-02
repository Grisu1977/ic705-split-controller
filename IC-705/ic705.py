'''
Docstring
Icom IC-705 Split Controller
Autor: Pascal Pfau (DH1PV)
Version: 1.0.0
Benötigte Bibliotheken:
- pip install pyserial
Einstellungen im TRX:
- CI-V USB Echo Back muss 'off' sein
Geplante Anpassungen:
- [✓] Hilfefenster / Infofenster
- [✓] Interaktionsmeldungen mit Messagebox
- [✓] Verbesserung der Fehleranpassung (Try/except)
- [✓] Umbau der bcd-Parsings
Zukünftige versionen:
- [] Linux Implementierung
- [] Optionale Speicherung der Fensterposition
- [] Hinzufügen eines Tray-Icons
- [] Manuelles setzen der TX/RX Frequenz mit berechnung und setzen der Offsetfrequenz
- [] Automatisches angleichen des Modes VFO A / VFO B bei wechsel
- [] Einschalten des Splitbetriebs bei Tracking Start
- [] Verbindung via WLAN
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

class CIV_Control:    
    def __init__(self):
        '''
        Initialisiert den Controller für den IC-705.
        Setzt Standardwerte für Adressen, Frequenzen und Offset.
        '''
        self.configfile = 'config.ini'
        self.freq_tracking = False
        self.connected = False
        self.lock = threading.Lock()
        # Default-Werte werden durch die Werte aus der 'config.ini' überschrieben
        self.serial_port = None # Serielle schnittstelle (Default)
        self.baud_rate = 19200 # Default TRX-Baudrate
        self.time_out = 0.1 # Timeout beim Connect zum TRX via Pyserial
        self.trx_Adresse = 0xa4 # Default TRX-Adresse
        self.controller_Adresse = 0xe0 # Muss in der regel nicht angepasst werden
        self.command_RX_freq = 0x03 # Hex CI-V Komando für die Frequenzabfrage (aktiver VFO)
        self.command_TX_freq = 0x25, 0x01 # Hex CI-V Komando für das setzen der TX-Frequenz (nicht aktiver VFO)
        self.offset = 287_500_000 # Default Offset für QO-100
        self.step = 10 # Default Schrittweite für manuelles nachjustieren

    def config_einlesen(self):
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
        config = configparser.ConfigParser()
        config['Transceiver'] = {
            'Last_COM_Port':str(self.serial_port),
            'Baud_Rate':self.baud_rate,
            'TRX_Adresse':f'{self.trx_Adresse:02x}' # Speichern als zweistellige HEX-Zahl
            }
        config['Offset'] = {'Last_Offset':self.offset,'Last_Step':self.step}
        # öffnen und schreiben der *.ini
        with open(self.configfile, 'w') as cf:
            cf.write( # Kommentarblock in der Datei
                '; #=================================================#\n' \
                '; | Konfigurationsdatei für IC-705 Split Controller |\n' \
                '; | Autor Pascal Pfau (DH1PV)                       |\n' \
                '; | Diese Datei wird automatisch erzeugt.           |\n' \
                '; | Manuelle Änderungen sind möglich,               |\n' \
                '; | geschen auf eigene Gefahr. Sollte Programm      |\n' \
                '; | nach änderungen nicht mehr wie erwartet laufen, |\n' \
                '; | kann diese Datei gefahrlos gelöscht werden.     |\n' \
                '; | Die Nutzung des Programms geschieht auf eigene  |\n' \
                '; | Gefahr.                                         |\n' \
                '; #=================================================#\n\n'
            )
            config.write(cf) # Konfiguration

    def connect(self):
        # Aufbau der verbindung zum TRX über Serielle Schnittstelle
        with self.lock:
            if self.serial_port is None:
                return False, 'Wähle einen Port'
            try:
                self.ic705 = serial.Serial(port=self.serial_port, baudrate=self.baud_rate, timeout=self.time_out, write_timeout=2)
                '''überprüfung ob verbindung korreckt hergestellt worden ist und ob gerät auch korrekt antwortet'''
                self.ic705.write([0xfe,0xfe,0xa4,0xe0,0x03,0xfd])
                time.sleep(0.05)
                dataTest = self.ic705.read_until()
                if len(dataTest) < 7:
                    self.ic705.close()
                    raise IOError('Keine korrekte Antwort vom Tranceiver.\
                                  \rIst der Tranceiver eingeschaltet?\
                                  \rIst der Richtige Port gewählt?')
                self.connected = True
                return True, None
            except serial.SerialTimeoutException as e:
                self.ic705 = None
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
                print(f'Fehler beim Trennen: {e}')

    def _message(self, txrx, bcd=None):
        '''Erstellen der CI-V Message'''
        msg=[]
        if txrx == 'tx': # Generierung der Message zum setzen der Sendefrequenz im TRX (Nicht aktiver VFO)
            msg = [0xfe, 0xfe, self.trx_Adresse, self.controller_Adresse, self.command_TX_freq[0], self.command_TX_freq[1], 0xfd]
            msg[6:6]=bcd
        else: # Generierung der Message zur abfrage der Empfangsfrequenz (Aktiver VFO)
            msg = [0xfe, 0xfe, self.trx_Adresse, self.controller_Adresse, self.command_RX_freq, 0xfd]
        return bytes(msg)

    def bcd_abfrage(self): # Binär Codierte Dezimalzahl
        '''Abfrage der Betriebs-Frequenz'''
        with self.lock:
            msg = self._message('rx')
            try:
                self.ic705.reset_input_buffer() # Löschen des empfangsbuffers
                self.ic705.write(msg) # Abfrage
                data = self.ic705.read_until(b'\xfd') # Lesen des unformatierten Datenstromes
                return data, None
            except Exception as e:
                return None, e

    def bcd_to_freq(self, bcd):
            '''Lesen und extrahieren der 5 Frequenz-Bytes'''
            try:
                if bcd[len(bcd)-7] in (0x03, 0x00) and bcd[len(bcd)-1] == 0xfd:
                    freq_bytes = bcd[len(bcd)-2:len(bcd)-7:-1] # Extration der 5 Frequenzbytes in richtiger reihenfolge
                    freq = freq_bytes.hex() 
                    return int(freq), None # Ausgabe der Frequenz in Hz als ganze Zahl
                else:
                    return None, None
            except Exception as e:
                return None, e

    def freq_to_bcd(self, freq):
        '''Konvertiert Frequenz in BCD-Format (5 Bytes für 10 Ziffern)'''
        freq = f'{freq:010d}' # Auffüllen mit nullen
        freq_bytes = [int(freq[i]+freq[i+1],16) for i in range(0, len(freq)-1, 2)] # Bildung BCD-Bytes: 2 Dezimalziffern = 1 Byte
        freq_bytes.reverse() # BCD-Bytes, niederwertige Dezimalziffern zuerst

        return freq_bytes

    def txfreq_write(self, bcd):
        '''Setzen der frequenz des nicht aktiven VFO'''
        with self.lock:
            msg = self._message('tx', bcd)
            try:
                self.ic705.write(msg)
            except Exception as e:
                print(f'Fehler beim Frequenz setzen: {e}')

    def abfrage_Ports(self):
        '''Abfrage "aller" aktiven Ports im System'''
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports[0].startswith('COM'):
            ports = sorted(ports, key=lambda x: int(x.replace('COM', '')))
        else:
            ports = sorted(ports, key=lambda x: int(x.replace(r'/dev/ttyS', '')))
        return ports

class CIV_GUI:
    def __init__(self):
        '''
        Initialisiert die GUI für den IC-705 Controller.
        Erstellt alle UI-Elemente und startet den Controller.
        '''
        self.fenster = tk.Tk()
        self.fenster.title('IC-705 Split Controler') # Name Titelleiste Fenster / Programmname
        self.fenster.resizable(False, False)
        self.fenster.protocol('WM_DELETE_WINDOW', self.closeFenster)
        self.control = CIV_Control()
        self.control.config_einlesen()
        self._setup_user_interface()
        self.start_ft = False

    def _setup_user_interface(self):

        style = ttk.Style()
        style.theme_use('clam')

        # Definition der Farben im Fenster
        self.bg_ausgabe = "#000000"
        self.fg_ausgabeRX = "#00ff00"
        self.fg_ausgabeTX = "#ff0000"
        self.bg_dunkel = "#1e293b"
        self.bg_mittel = "#334155"
        self.rahmen_hell = "#dbdbbd"
        self.schrift = "#ffffff"

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
                                text='Icom IC-705 Split Controler', # Name des Programms
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
                                    width=10,
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
        self.etOffset_var = tk.StringVar(value=self.control.offset)
        self.etOffset = tk.Entry(self.frOffset_uli,
                                 font=('Courier New', 12),
                                 width=12,
                                 validate='key',
                                 validatecommand=(self.frOffset_uli.register(self._filter_etOffset), '%P'),
                                 textvariable=self.etOffset_var)
        self.etOffset.grid(row=0, column=0, padx=5, pady=5)
        self.etOffset.bind('<Return>',self._commit_etOffset)
        self.etOffset.bind('<FocusOut>',self._commit_etOffset)

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
        self.cbStep_var = tk.StringVar(value=self.control.step)
        self.cbStep = ttk.Combobox(self.frOffset_ure,
                                      values=[1,10,100,1_000,10_000,100_000,1_000_000],
                                      textvariable=self.cbStep_var,
                                      state='readonly',
                                      width=12)
        self.cbStep.grid(row=0, column=0)
        self.cbStep.bind('<<ComboboxSelected>>', self._cbStep_auswahl)

    def _filter_etOffset(self, value):
        '''Filtern der Benutzereingaben'''
        return value == '' or value == '-' or value.lstrip("-").isdigit()

    def _commit_etOffset(self, event=None): # event wird von bind immer mitgeliefert aber hier nicht verwendet
        '''einstellen des Benutzeroffset'''
        raw_etOffset = self.etOffset_var.get()
        try:
            value_etOffst = int(raw_etOffset)
            if -1_000_000_000 < value_etOffst < 1_000_000_000:
                self.control.offset=value_etOffst
                self.refresh_lbRXTX_Anzeige() # Aktualliesierung der Anzeige
            else:
                raise ValueError
        except ValueError as e:
            self.etOffset.focus_set()
            self.etOffset.select_range(0, tk.END)
            messagebox.showerror(title='Fehler', message='Bitte eine gültige Frequenz in Herz eingeben\nNur Ziffern 0 ... 9')
            return "break"

    def _etOffset_pm(self, plusminus):
        '''Feinjustierung des Offsets'''
        var = int(self.etOffset_var.get())
        step = int(self.control.step)
        self.etOffset_var.set(var + step * plusminus) # einstellen der richtung Plus / Minus
        self.control.offset=int(self.etOffset_var.get())
        self.refresh_lbRXTX_Anzeige() # Aktuallisierung der Anzeige

    def _cbPorts_auswahl(self, *args):
        '''einstellen eines Ports'''
        self.control.serial_port = None if self.cbPorts_var.get() == 'None' else self.cbPorts_var.get()

    def _refresh_ports(self):
        '''Aktuallisierung der Ports'''
        ports = self.control.abfrage_Ports()
        self.cbPorts.configure(values=ports)

    def _cbStep_auswahl(self,*args):
        '''Auswahl der Offset schrittweite'''
        self.control.step = self.cbStep.get()

    def _tracking_on(self):
        '''Aktiviert das automatische Nachstellen des nicht aktiven VFO (Tracking)'''
        self.control.freq_tracking = True
        self.buTracking.config(background="#ffdd00", text='Tracking Stop', command=self._tracking_off)
        self.refresh_lbRXTX_Anzeige() # Aktuallisierung der Anzeige

    def _tracking_off(self):
        '''Deaktiviert das automatische Nachstellen des nicht aktiven VFO (Tracking)'''
        self.control.freq_tracking = False
        self.buTracking.config(background="#0000ff", text='Tracking Start', command=self._tracking_on)
        self.lbTX_Anzeige_text.set(value='off') # TX-Anzeige auf 'off' schalten

    def start_frequenz_update_thread(self):
        '''Starten der kontinuirlichen Frequenzabfrage'''
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
        '''Stoppen der kontinuirlichen Frequenzabfrage'''
        self.start_ft = False
        if self.control.freq_tracking:
            self._tracking_off()
        self.control.disconnect()
        '''Einstellen der Bedienelemente'''
        self.cbPorts.config(state='readonly')
        self.buPorts_refresh.config(state='normal')
        self.buTracking.config(state='disabled')
        self.buVerbinden.config(text='Verbinden', command=self.start_frequenz_update_thread, background='#00ff00')
        self.status_indikator.itemconfig(self.status_indikator_oval, fill='#ff0000')
        self.lbRX_Anzeige_text.set(value='off') # Anzeige auf "off" stellen

    def refresh_lbRXTX_Anzeige(self):
        '''Aktuallisiert die TX-Anzeige'''
        if self.start_ft and self.control.freq_tracking:
            rx = self.freq_update()
            tx = rx + self.control.offset
            self.update_lbRXTX_Anzeige(freqrx=rx, freqtx=tx, refresh=True)

    def update_lbRXTX_Anzeige(self, freqrx, freqtx=None, refresh = False):
        '''Anzeige der Frequenzen'''
        if self.start_ft and self.control.connected:
            mhzrx = freqrx / 1_000_000
            self.lbRX_Anzeige_text.set(value=f'{mhzrx:.6f} MHz')
            if self.control.freq_tracking and not freqtx is None:
                mhztx = freqtx / 1_000_000
                self.lbTX_Anzeige_text.set(value=f'{mhztx:.6f} MHz')
                if refresh:
                    self.txfreq_set(freqtx)

    def freq_update(self):
        '''Abfrage der RX-Frequenz für die Anzeige und Berechnung der TX-Frequenz'''
        if self.start_ft and self.control.connected:
            freq_err = None
            bcd_err = None
            freq = None
            while freq is None and freq_err is None:
                bcd = [0xfe, 0xfe, 0xe0, 0xa4, 0xfb, 0xfd]
                while bcd[4] == 0xfb:
                    bcd, bcd_err = self.control.bcd_abfrage()
                    if bcd[4] == 0xfb:
                        pass
                    print(f'bcd: {bcd}')
                if bcd_err is None:
                    freq, freq_err = self.control.bcd_to_freq(bcd)
                    print(f'freq: {freq}')
                    if freq_err is None and freq:
                        return freq

            msg = 'Die Verbindung wird getrennt. Überprüfe den Tranceiver\n'
            if bcd_err:
                msg += f'\nbcd Fehler: {bcd_err}'
            if freq_err:
                msg += f'\nfreq Fehler: {freq_err}'
            messagebox.showerror(title='Fehler', message=msg)
            return None

    def txfreq_set(self, freqtx):
        '''Setzen der Sendefrequenz (Nicht Aktiver VFO) im Funkgerät'''
        if self.start_ft and self.control.connected:
            bcd = self.control.freq_to_bcd(freqtx)
            self.control.txfreq_write(bcd)

    def frequenz_update_thread(self):
        '''Updateschleife für Anzeige und Tracking'''
        freqrx_alt = 0
        while self.start_ft and self.control.connected:
            freqrx = self.freq_update()
            if freqrx is None:
                self.stop_frequenz_update_thread() # Bei Fehler Stopp des UpdateThreads
            else:
                diff = freqrx - freqrx_alt
                if abs(diff) > 0: # einstellen der TX-Frequenz nur wenn es differenz gibt
                    if self.control.freq_tracking:
                        freqtx = freqrx + self.control.offset # berechnung der TX-Frequenz
                        self.lbRX_Anzeige.after(0, self.update_lbRXTX_Anzeige, freqrx, freqtx)
                        self.txfreq_set(freqtx)
                    else:
                        self.lbRX_Anzeige.after(0, self.update_lbRXTX_Anzeige, freqrx)
                freqrx_alt = freqrx
                for i in range(10):
                    '''Warteschleife'''
                    if not self.start_ft or not self.control.connected: # überprüfung auf abbruch beim warten
                        break
                    time.sleep(0.01)

    def hilfe(self):
        '''Generierung eines Hilfe- und Infofensters'''
        info = '''\
Icom IC-705 Split Controller v1.0.0
Autor: Pascal Pfau (DH1PV)
eMail: dh1pv@darc.de
©2026\n
Funktionen:
- Frequenznachführung für Crossbandbetrieb
- Offset-Korrektur
- Feinabstimmung der TX-Frequenz
- Speicherung der Einstellungen\n
Release Notes:
02-01-2026
- Erste Version

Disclaimer:
Die benutzung des Programms geschied auf eigene Gefahr. Für Schäden an Geräten (Computer, TRX, etc.) \
und Software, sowie Datenverlussten, übernehme ich keinerlei haftung.\n
Erste schritte:
Nach dem Start als erstes den richtigen Seriellen Port auswählen. Meist in der Geräteverwaltung mit \
CI-V gekennzeichnet. Danach auf Verbinden klicken. Jetzt wird die Aktuelle RX-Frequenz angezeigt. Zum \
Nachführen der TX-Frequenz Tracking Start klicken. Nun wird auch die Aktuelle TX-Frequenz angezeigt und \
automatisch anhand des aktuell eingestelltem Offsets gesetzt.
Das Offset kann manuell in Herz eingegeben werden oder aber in einzelschritten, einstellbar unter \
Schrittweite, mit "+" und "-" eingestellt werden.\n
! ! ! ACHTUNG ! ! !
Damit das Programm richtig funktioniert ist es erforderlich das sowol 
MENU >> Connectors >> CI-V >> CI-V USB Echo Back = OFF
und bei nutzung via Bluetooth
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

    def closeFenster(self):
        '''Schließen / Beenden des Programms'''
        if self.start_ft:
            self.stop_frequenz_update_thread()
        self.fenster.destroy()
        self.control.config_schreiben()


gui=CIV_GUI()
gui.fenster.mainloop()
