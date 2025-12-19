import time
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk
import threading
import configparser
from pathlib import Path

class CIV_Control:    
    def __init__(self):
        self.configfile = 'config.ini'
        self.serial_port = None
        self.freq_tracking = False
        self.connected = False
        self.baud_rate = 19200
        self.time_out = 0.05
        self.trx_Adresse = 0xa4
        self.controler_Adresse = 0xe0
        self.command_RX_freq = 0x03
        self.command_TX_freq = 0x25, 0x01
        self.offset = 287_500_000
        self.step = 10

    def config_einlesen(self):
        config = configparser.ConfigParser()
        if Path(self.configfile).is_file():
            config.read(self.configfile)
            if not config['Transceiver']['Last_COM_Port'] == 'None':
                self.serial_port = config['Transceiver']['Last_COM_Port']
            self.baud_rate = int(config['Transceiver']['Baud_Rate'])
            self.trx_Adresse = int(config['Transceiver']['TRX_Adresse'],16)
            self.offset = int(config['Offset']['Last_Offset'])
            self.step = int(config['Offset']['Last_Step'])

    def config_schreiben(self):
        config = configparser.ConfigParser()
        config['Transceiver'] = {
            'Last_COM_Port':str(self.serial_port),
            'Baud_Rate':self.baud_rate,
            'TRX_Adresse':f'{self.trx_Adresse:02x}'
            }
        config['Offset'] = {'Last_Offset':self.offset,'Last_Step':self.step}
        with open(self.configfile, 'w') as cf:
            config.write(cf)

    def connect(self, port, baud, timeout):
        try:
            self.ic705=serial.Serial(port=port,baudrate=baud,timeout=timeout)
            self.connected = True
            return True, None
        except Exception as e:
            self.ic705 = None
            return False, e
        
    def disconnect(self):
        pass

    def bcd_abfrage(self):
        msg=self._message('rx')
        cmd=bytes(msg)
        try:
            self.ic705.write(cmd)
            time.sleep(0.05)
            data=self.ic705.read_until().hex(sep=' ')
            return data
        except Exception as e:
            print(f'Fehler bei der bcd Abfrage: {e}')
        
    
    def _message(self, txrx, bcd=None):
        msg=[]
        if txrx == 'tx':
            msg = [0xfe, 0xfe, self.trx_Adresse, self.controler_Adresse, self.command_TX_freq[0], self.command_TX_freq[1], 0xfd]
            msg[6:6]=bcd
        else:
            msg = [0xfe, 0xfe, self.trx_Adresse, self.controler_Adresse, self.command_RX_freq, 0xfd]
        return msg
    
    # extration der Frequenz
    def bcd_to_freq(self, bcd=str):
            ldata = bcd.split(' ')
            for i in range(len(ldata)-1, -1, -1):
                if ldata[i] == 'fd':
                    fd = i
                    break
            qrg_bytes = [ldata[i] for i in range(fd-5, fd)] # Extration der fünf Frequenzbytes
            qrg_bytes.reverse() 
            qrg = ''.join(qrg_bytes) # Liste wieder umwandeln zu einem String
            return int(qrg) # Ausgabe der Frequenz in Hz als ganze Zahl
    
    def freq_to_bcd(self, freq):
        freq = f'{freq:010d}'#f'{int(freq * 1e6):010d}'
        freq_bytes = [int(freq[i]+freq[i+1],16) for i in range(0, len(freq)-1, 2)]
        freq_bytes.reverse()
        return freq_bytes
    
    def freq_setzen(self, bcd=list):
        msg = self._message('tx', bcd)
        msg = bytes(msg)
        try:
            self.ic705.write(msg)
        except Exception as e:
            print(f'Fehler beim Frequenz setzen: {e}')

class CIV_GUI:
    def __init__(self):
        self.fenster = tk.Tk()
        self.fenster.title('CI-V Controll') # Name Titelleiste Fenster / Programmname
        self.fenster.resizable(False, False)
        self.fenster.protocol('WM_DELETE_WINDOW', self.schliessen)
        self.control = CIV_Control()
        self.control.config_einlesen()
        self.setup_user_interface()
        self.start_ft = False

    def setup_user_interface(self):

        style = ttk.Style()
        style.theme_use('clam')

        # Definition der Farben im Fenster
        bg_ausgabe = "#000000"
        fg_ausgabeRX = "#00ff00"
        fg_ausgabeTX = "#ff0000"
        bg_dunkel = "#1e293b"
        bg_mittel = "#334155"
        rahmen_hell = "#dbdbbd"
        schrift = "#ffffff"

        self.fenster.configure(background=bg_dunkel) # Setzen der Hintergrundfarbe

        self.frTitel = tk.Frame(self.fenster,
                                background=bg_mittel,
                                highlightbackground=rahmen_hell,
                                highlightthickness=1,
                                width=425,
                                height=50)
        self.frTitel.grid(row=0, column=0, padx=5, pady=5, sticky='ew')
        self.frTitel.grid_propagate(False)
        self.frTitel.columnconfigure(0,weight=0)
        self.frTitel.columnconfigure(1,weight=1)
        self.frTitel.rowconfigure(0, weight=1)
        self.lbTitel = tk.Label(self.frTitel,
                                background=bg_mittel, # Hintergrund
                                text='Icom Split Controler', # Name des Programms
                                font=(None, 16, 'bold'),
                                foreground=schrift) # Schriftfarbe
        self.lbTitel.grid(row=0, column=0)
        self.frTitel_re = tk.Frame(self.frTitel, background=bg_mittel)
        self.frTitel_re.grid(row=0, column=1, sticky='e')
        self.status_indikator = tk.Canvas(self.frTitel_re,
                                          width=15,
                                          height=15,
                                          bg=bg_mittel,
                                          highlightthickness=0)
        self.status_indikator.grid(row=0, column=0)
        self.status_indikator_oval = self.status_indikator.create_oval(0, 0, 15, 15, fill='#ff0000')
        self.buHilfe = tk.Button(self.frTitel_re,
                                 command=self.hilfe,
                                 text='?', 
                                 font=(None, 13, 'bold'),
                                 foreground='red',
                                 background=bg_mittel,
                                 borderwidth=0,
                                 highlightthickness=0,
                                 activebackground=bg_mittel)
        self.buHilfe.grid(row=0, column=1)

        self.frVerbinden = tk.Frame(self.fenster, background=bg_dunkel)
        self.frVerbinden.grid(row=1, column=0, padx=5, pady=2, sticky='w')
        self.lbPorts = tk.Label(self.frVerbinden,
                               relief='flat',
                               background=bg_dunkel,
                               text='COM Port:',
                               foreground=schrift)
        self.lbPorts.grid(row=0, column=0)
        self.cbPorts_var = tk.StringVar()
        self.cbPorts = ttk.Combobox(self.frVerbinden,
                                    width=10,
                                    values=self._abfrage_Ports(),
                                    state='readonly',
                                    textvariable=self.cbPorts_var
                                    )
        self.cbPorts.grid(row=0, column=1, padx=5, pady=0)
        self.cbPorts_var.trace_add('write', self._cbPorts_auswahl)
        self.cbPorts_var.set(str(self.control.serial_port))
        self.buPorts_refresh = tk.Button(self.frVerbinden,
                                         command=lambda:self._abfrage_Ports(refresh=True),
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
                                    command=self._tracking)
        self.buTracking.grid(row=0, column=4, padx=5, pady=0)

        self.frAnzeige = tk.Frame(self.fenster,
                                  background=bg_mittel,
                                  highlightbackground=rahmen_hell,
                                  highlightthickness=1)
        self.frAnzeige.grid(row=2, column=0, padx=5, pady=5, sticky='ew')
        self.frAnzeige.rowconfigure(0, weight=1)
        self.frAnzeige.rowconfigure(1, weight=1)
        self.frAnzeige.columnconfigure(0, weight=1, uniform='col')
        self.frAnzeige.columnconfigure(1, weight=1, uniform='col')
        self.lbRX = tk.Label(self.frAnzeige,
                             text='RX Frequenz',
                             font=(None, 12, 'bold'),
                             background=bg_mittel,
                             foreground=fg_ausgabeRX)
        self.lbRX.grid(row=0, column=0, padx=2, pady=0)
        self.lbTX = tk.Label(self.frAnzeige,
                             text='TX Frequenz',
                             font=(None, 12, 'bold'),
                             background=bg_mittel,
                             foreground=fg_ausgabeTX)
        self.lbTX.grid(row=0, column=1, padx=2, pady=0)
        self.lbRX_Anzeige_text = tk.StringVar(value='off')
        self.lbRX_Anzeige = tk.Label(self.frAnzeige,
                                     relief='sunken',
                                     background=bg_ausgabe,
                                     foreground=fg_ausgabeRX,
                                     textvariable=self.lbRX_Anzeige_text,
                                     font=('Courier New', 15),
                                     width=15,
                                     height=1)
        self.lbRX_Anzeige.grid(row=1, column=0, padx=2, pady=2, sticky='ew')
        self.lbTX_Anzeige_text = tk.StringVar(value='off')
        self.lbTX_Anzeige = tk.Label(self.frAnzeige,
                                     relief='sunken',
                                     background=bg_ausgabe,
                                     foreground=fg_ausgabeTX,
                                     textvariable=self.lbTX_Anzeige_text,
                                     font=('Courier New', 15),
                                     width=15,
                                     height=1)
        self.lbTX_Anzeige.grid(row=1, column=1, padx=2, pady=2, sticky='ew')

        self.frOffset = tk.Frame(self.fenster,
                                 background=bg_mittel,
                                 highlightbackground=rahmen_hell,
                                 highlightthickness=1)
        self.frOffset.grid(row=3, column=0, padx=5, pady=5, sticky='ew')
        self.frOffset.columnconfigure(0, weight=1, uniform='col')
        self.frOffset.columnconfigure(1, weight=1, uniform='col')
        self.lbOffset = tk.Label(self.frOffset,
                                 background=bg_mittel,
                                 text='Frequenz - Offset (Hz)',
                                 font=(None, 8),
                                 foreground=schrift)
        self.lbOffset.grid(row=0, column=0, sticky='w')
        self.lbOffset_schritt = tk.Label(self.frOffset,
                                         background=bg_mittel,
                                         text='Schrittweite',
                                         font=(None, 8),
                                         foreground=schrift)
        self.lbOffset_schritt.grid(row=0, column=1, sticky='w')
        self.frOffset_uli = tk.Frame(self.frOffset,
                                     background=bg_mittel)
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
        self.cbSchritt = ttk.Combobox(self.frOffset_ure,
                                      values=[1,10,100,1_000,10_000,100_000,1_000_000],
                                      state='readonly',
                                      width=12)
        self.cbSchritt.grid(row=0, column=0)
        self.cbSchritt.current(0)

    def _filter_etOffset(self, value):
        return value == '' or value == '-' or value.lstrip("-").isdigit()
    
    def _commit_etOffset(self, event=None):
        raw_etOffset = self.etOffset_var.get()
        try:
            value_etOffst = int(raw_etOffset)
            if -1_000_000_000 < value_etOffst < 1_000_000_000:
                self.control.offset=value_etOffst
                self.update_lbRXTX_Anzeige(True)
            else:
                raise ValueError
        except ValueError as e:
            self.etOffset.focus_set()
            self.etOffset.select_range(0, tk.END)
            print(e)
            return "break"

    def _etOffset_pm(self, plusminus):
        var = int(self.etOffset_var.get())
        self.etOffset_var.set(var + self.control.step * plusminus)
        self.control.offset=int(self.etOffset_var.get())
        self.update_lbRXTX_Anzeige(True)

    def _cbPorts_auswahl(self, *args):
        self.control.serial_port = self.cbPorts_var.get()

    def _abfrage_Ports(self, refresh=False):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports[0].startswith('COM'):
            ports = sorted(ports, key=lambda x: int(x.replace('COM', '')))
        else:
            ports = sorted(ports, key=lambda x: int(x.replace(r'/dev/ttyS', '')))
        if refresh:
            self.cbPorts.configure(values=ports)
        else:
            return ports




    def _tracking(self):
        if not self.control.freq_tracking:
            self.control.freq_tracking = True
            self.buTracking.config(background="#ffdd00", text='Tracking Stop')
            self.update_lbRXTX_Anzeige(True)
        else:
            self.control.freq_tracking = False
            self.buTracking.config(background="#0000ff", text='Tracking Start')
            self.lbTX_Anzeige_text.set(value='off')

    def start_frequenz_update_thread(self):
        if not self.start_ft:
            connect, err = self.control.connect(self.control.serial_port, self.control.baud_rate, self.control.time_out)
            if not connect:
                print(f'Verbindungsfehler * * * {err} * * *')
            else:
                self.ft = threading.Thread(target=self.frequenz_update_thread)            
                self.start_ft = True
                self.buVerbinden.config(text='Trennen', command=self.stop_frequenz_update_thread, background='#ff0000')
                self.status_indikator.itemconfig(self.status_indikator_oval, fill='#00ff00')
                self.buTracking.config(state='normal')
                self.ft.start()






    def stop_frequenz_update_thread(self):
        self.start_ft = False
        self.ft.join(1)
        self.buVerbinden.config(text='Verbinden', command=self.start_frequenz_update_thread, background='#00ff00')
        self.status_indikator.itemconfig(self.status_indikator_oval, fill='#ff0000')
        self.lbRX_Anzeige_text.set(value='off')
        if self.control.freq_tracking:
            self._tracking()
        self.buTracking.config(state='disabled')
        self.control.ic705.close()



    def update_lbRXTX_Anzeige(self, freqrx, freqtx=None, refresh=False):
        if not self.start_ft:
            return
        mhzrx = freqrx / 1_000_000
        self.lbRX_Anzeige_text.set(value=f'{mhzrx:.6f} MHz')
        if self.control.freq_tracking and not freqtx == None:
            mhztx = freqtx / 1_000_000
            self.lbTX_Anzeige_text.set(value=f'{mhztx:.6f} MHz')




    def freq_update(self): # Abfrage der Empfangsfrequenz
        if not self.start_ft:
            print('freq update')
            return
        bcd = self.control.bcd_abfrage()
        freq = self.control.bcd_to_freq(bcd)
        return freq

    def freq_set(self, freqtx): # Setzender Sendefrequenz im Funkgerät
        bcd=self.control.freq_to_bcd(freqtx)
        if not self.start_ft:
            print('freq set')
            return
        self.control.freq_setzen(bcd)

    def frequenz_update_thread(self):
        freqrx_alt = 0
        while self.start_ft:
            print('aktiv')
            freqrx = self.freq_update()
            freqtx = freqrx + self.control.offset
            diff = freqrx - freqrx_alt
            if self.control.freq_tracking:
                self.lbRX_Anzeige.after(0, self.update_lbRXTX_Anzeige, freqrx, freqtx)
                if abs(diff) > 0:
                    self.freq_set(freqtx)
            else:
                self.lbRX_Anzeige.after(0, self.update_lbRXTX_Anzeige, freqrx)
            freqrx_alt = freqrx
            for i in range(10):
                if not self.start_ft:
                    print('stop', i)
                    break
                time.sleep(0.025)
        print('beendet')

    def hilfe(self):
        pass

    def schliessen(self):
#        if self.start_ft:
#            self.stop_frequenz_update_thread()
        self.start_ft = False
        self.ft.join(1)
        self.control.ic705.close()        
        time.sleep(1)
        self.fenster.destroy()
        self.control.config_schreiben()
        print('Config.ini Geschrieben')


gui=CIV_GUI()
gui.fenster.mainloop()
print('Programm ENDE')
