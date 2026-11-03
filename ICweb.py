"""IC-7300 web kontrola: CI-V preko USB serijskog porta + web sucelje (aiohttp/WebSocket).
+ waterfall (scope) i audio RX u browseru.
+ DODANO: filteri FIL1/2/3, 8 CW makronaredbi, prekid CW predaje, direktni unos CW teksta.
  Web sucelje za nove funkcije je na adresi  /cw  (index.html ostaje netaknut).

Pokretanje:  python server.py --port COM3 --password tvojalozinka --audio-out "USB Audio CODEC"
Potrebno:    pip install pyserial aiohttp numpy sounddevice

WebSocket protokol (binarne poruke, prvi bajt = tip):
  0x01 + 8 bajtova (2x uint32 BE: f_low, f_high u Hz) + N bajtova piksela (0-160)  -> waterfall red
  0x02 + N bajtova u-law audio (8 kHz, mono)                                       -> zvuk u browseru
  0x03 + N bajtova u-law audio (klijent -> radio, samo dok je PTT aktivan)         -> TX

Nove WS naredbe (tekst JSON {"k":..., "v":...}):
  {"k":"filter","v":1|2|3}   postavi FIL1/2/3 za trenutni mod (06h + filter bajt)
  {"k":"cw","v":"<tekst>"}   predaja CW teksta (17h, do 30 znakova po okviru);
                             ako nije CW mod, automatski prebaci na CW
  {"k":"cw_stop"}            prekini CW makro predaju (17h 00h) + vrati u RX (1Ch 00h 00h)
Makronaredbe: GET/POST /api/macros (sprema se u ic7300_macros.json pored skripte)
"""
import argparse, asyncio, base64, json, threading, time
import datetime
from pathlib import Path
import serial, struct, ssl, re, os, tempfile, shutil
from aiohttp import web
try:
    import numpy as np
    import sounddevice as sd
except ImportError:          # audio je opcionalan
    np = sd = None

VERSION = 'v2.1 (CW makronaredbe + filteri + /cw konzola)'

CTRL = 0xE0
MODES = {0: 'LSB', 1: 'USB', 2: 'AM', 3: 'CW', 4: 'RTTY', 5: 'FM', 7: 'CW-R', 8: 'RTTY-R'}
MODE_ID = {v: k for k, v in MODES.items()}
LEVELS = {'af': 0x01, 'rf': 0x02, 'sql': 0x03, 'power': 0x0A, 'cwspd': 0x0C}   # cmd 14 xx
FLAGS = {'nb': 0x22, 'nr': 0x40, 'anf': 0x41}                    # cmd 16 xx, 0/1
METERS = {0x02: 'smeter', 0x11: 'pwr_meter', 0x12: 'swr'}        # cmd 15 xx

MACRO_PATH = Path(__file__).with_name('ic7300_macros.json')
ADIF_PATH = Path(__file__).with_name('ic7300_log.adi')

BANDS = [(1840000, '160M'), (3570000, '80M'), (5351500, '60M'), (7100000, '40M'),
         (10100000, '30M'), (14000000, '20M'), (18100000, '17M'), (21000000, '15M'),
         (24900000, '12M'), (28000000, '10M'), (50000000, '6M'), (70000000, '4M')]


def adif_band(freq):
    for f, b in reversed(BANDS):
        if freq >= f:
            return b
    return ''


def adif_record(call, rst, rstr, freq, mode, mycall='', grid='', name='', qth=''):
    now = datetime.datetime.utcnow()
    fields = [('call', call), ('mode', mode),
              ('rst_sent', rst), ('rst_rcvd', rstr), ('name', name), ('qth', qth),
              ('qso_date', now.strftime('%Y%m%d')), ('time_on', now.strftime('%H%M%S')),
              ('band', adif_band(freq).lower()), ('freq', '%.6f' % (freq / 1e6)),
              ('station_callsign', mycall), ('my_gridsquare', grid)]
    return ''.join('<%s:%d>%s ' % (k.upper(), len(v.encode('utf-8')), v) for k, v in fields if v) + '<EOR>\n'


def adif_append(call, rst, rstr, freq, mode, mycall='', grid='', name='', qth=''):
    with ADIF_LOCK:
        return _adif_append_unlocked(call, rst, rstr, freq, mode, mycall, grid, name, qth)

def _adif_append_unlocked(call, rst, rstr, freq, mode, mycall='', grid='', name='', qth=''):
    """Dopise QSO u ADIF log; vraca ukupan broj zapisa."""
    rec = adif_record(call, rst, rstr, freq, mode, mycall, grid, name, qth)
    fresh = not ADIF_PATH.exists()
    with open(ADIF_PATH, 'a', encoding='utf-8') as f:
        if fresh:
            f.write('ADIF export from IC-7300 Web Remote\n'
                    '<ADIF_VER:5>3.1.4\n<PROGRAMID:10>IC7300WEB\n<EOH>\n\n')
        f.write(rec)
    n = 0
    try:
        n = sum(1 for line in open(ADIF_PATH, encoding='utf-8') if '<eor>' in line.lower())
    except Exception:
        pass
    return n


# ADIF parser radi s bajtovima: ADIF duljine su UTF-8 bajtovi, ne znakovi.
ADIF_LOCK = threading.RLock()
EDIT_FIELDS = ('CALL','RST_SENT','RST_RCVD','NAME','QTH','QSO_DATE','TIME_ON','FREQ','BAND','MODE')

def adif_parse(raw):
    tags = []
    pos = 0
    pattern = re.compile(rb'<([A-Za-z_][A-Za-z_0-9]*)(?::([0-9]+)(?::[^>]*)?)?>', re.I)
    while pos < len(raw):
        m = pattern.search(raw, pos)
        if not m:
            break
        tag = m.group(1).decode('ascii').upper()
        if tag in ('EOR','EOH'):
            tags.append((tag, b''))
            pos = m.end()
            continue
        if m.group(2) is None:
            pos = m.end()
            continue
        size = int(m.group(2))
        if size > 1000000 or m.end()+size > len(raw):
            raise ValueError('Neispravna ADIF duljina polja')
        tags.append((tag, raw[m.end():m.end()+size]))
        pos = m.end()+size
    return tags

def adif_load():
    if not ADIF_PATH.exists():
        return [], []
    raw = ADIF_PATH.read_bytes()
    if not raw.strip():
        return [], []
    header, records, current = [], [], []
    for tag, value in adif_parse(raw):
        if tag == 'EOH':
            header = current
            current = []
        elif tag == 'EOR':
            records.append(current)
            current = []
        else:
            current.append((tag, value))
    if current:
        raise ValueError('ADIF sadrži nedovršen zapis')
    return header, records

def adif_field(record, tag):
    return next((v.decode('utf-8', 'replace') for k,v in record if k == tag), '')

def adif_serialize(header, records):
    def encode_fields(fields):
        return b' '.join(b'<'+k.encode('ascii')+b':'+str(len(v)).encode('ascii')+b'>'+v for k,v in fields)
    if header:
        out = b'ADIF export from IC-7300 Web Remote\n'+encode_fields(header)+b'\n<EOH>\n\n'
    else:
        out = b'ADIF export from IC-7300 Web Remote\n<ADIF_VER:5>3.1.4\n<PROGRAMID:10>IC7300WEB\n<EOH>\n\n'
    for rec in records:
        out += encode_fields(rec)+b' <EOR>\n'
    return out

def adif_update(index, changes):
    with ADIF_LOCK:
        header, records = adif_load()
        if not (0 <= index < len(records)):
            raise ValueError('Veza nije pronađena')
        rec = records[index]
        for tag, value in changes.items():
            if tag not in EDIT_FIELDS:
                continue
            val = str(value).strip()
            if tag == 'CALL':
                val = val.upper()
                if not val or len(val) > 10:
                    raise ValueError('Callsign mora imati 1–10 znakova')
            if tag in ('RST_SENT','RST_RCVD') and len(val) > 3:
                raise ValueError('RST ima najviše 3 znaka')
            if tag == 'NAME' and len(val) > 10 or tag == 'QTH' and len(val) > 15:
                raise ValueError('Predugačko polje Name/QTH')
            if tag == 'QSO_DATE' and not re.fullmatch(r'\d{8}', val):
                raise ValueError('Date must be YYYYMMDD')
            if tag == 'TIME_ON' and not re.fullmatch(r'\d{4}(?:\d{2})?', val):
                raise ValueError('Vrijeme mora biti HHMM ili HHMMSS')
            if tag == 'FREQ':
                try:
                    if not (0 < float(val) < 1000): raise ValueError()
                except ValueError: raise ValueError('Frekvencija mora biti u MHz')
            rec = [(k,v) for k,v in rec if k != tag]
            if val:
                rec.append((tag, val.encode('utf-8')))
        records[index] = rec
        # Sačuvaj originalni log prije zamjene, a novu verziju zapiši atomarno.
        tmp = None
        try:
            with tempfile.NamedTemporaryFile(dir=str(ADIF_PATH.parent), delete=False) as f:
                tmp = Path(f.name)
                f.write(adif_serialize(header, records))
                f.flush()
                os.fsync(f.fileno())
            shutil.copy2(ADIF_PATH, ADIF_PATH.with_suffix('.adi.bak'))
            os.replace(tmp, ADIF_PATH)
        finally:
            if tmp and tmp.exists(): tmp.unlink()


def adif_delete(index):
    """Obriši jedan QSO uz backup i atomarnu zamjenu ADIF datoteke."""
    with ADIF_LOCK:
        header, records = adif_load()
        if not (0 <= index < len(records)):
            raise ValueError('Veza nije pronađena')
        del records[index]
        tmp = None
        try:
            with tempfile.NamedTemporaryFile(dir=str(ADIF_PATH.parent), delete=False) as f:
                tmp = Path(f.name)
                f.write(adif_serialize(header, records))
                f.flush()
                os.fsync(f.fileno())
            shutil.copy2(ADIF_PATH, ADIF_PATH.with_suffix('.adi.bak'))
            os.replace(tmp, ADIF_PATH)
        finally:
            if tmp and tmp.exists():
                tmp.unlink()


def norm8(m, limit=120, count=8):
    """Normaliziraj listu na tocno count stringova (makro M1..)."""
    m = [str(x)[:limit] for x in m][:count]
    return m + [''] * (count - len(m))


def norm_names(n, count=8):
    """Imena makroa: do 8 znakova; po defaultu M1..M(count)."""
    n = [str(x)[:8] for x in n][:count]
    return n + ['M%d' % (i + 1) for i in range(len(n), count)]


def load_macros():
    """Vraca (tekstovi, imena) - staru datoteku prihvaca."""
    try:
        d = json.loads(MACRO_PATH.read_text(encoding='utf-8'))
        return norm8(d.get('macros', []), count=15), \
            norm_names(d.get('names', []), count=15), \
            str(d.get('mycall', ''))[:12], str(d.get('gridsquare', ''))[:8]
    except Exception:
        return [''] * 15, norm_names([], count=15), '', ''


def save_macros(m, n=None, mycall='', grid=''):
    try:
        MACRO_PATH.write_text(json.dumps({'macros': norm8(m, count=15),
                                          'names': norm_names(n or [], count=15),
                                          'mycall': str(mycall)[:12],
                                          'gridsquare': str(grid)[:8]},
                                         ensure_ascii=False, indent=1), encoding='utf-8')
    except Exception:
        pass


def bcd_freq(f):
    s = f"{int(f):010d}"
    return bytes(int(s[i:i + 2], 16) for i in (8, 6, 4, 2, 0))


def parse_freq(b):
    return int(''.join(f"{x:02X}" for x in reversed(b)))


def bcd_lvl(v):
    s = f"{max(0, min(255, int(v))):04d}"
    return bytes([int(s[:2], 16), int(s[2:], 16)])


def parse_lvl(b):
    return int(f"{b[0]:02X}{b[1]:02X}")


class Radio:
    MORSE = {'A': '.-', 'B': '-...', 'C': '-.-.', 'D': '-..', 'E': '.', 'F': '..-.',
             'G': '--.', 'H': '....', 'I': '..', 'J': '.---', 'K': '-.-', 'L': '.-..',
             'M': '--', 'N': '-.', 'O': '---', 'P': '.--.', 'Q': '--.-', 'R': '.-.',
             'S': '...', 'T': '-', 'U': '..-', 'V': '...-', 'W': '.--', 'X': '-..-',
             'Y': '-.--', 'Z': '--..', '0': '-----', '1': '.----', '2': '..---',
             '3': '...--', '4': '....-', '5': '.....', '6': '-....', '7': '--...',
             '8': '---..', '9': '----.', '.': '.-.-.-', ',': '--..--', '?': '..--..',
             '/': '-..-.', '=': '-...-', '+': '.-.'}

    def __init__(self, port, baud, addr, keyline='RTS'):
        self.port = port
        self.baud = baud
        self.addr = addr
        # Na IC-7300u se preko DTR/RTS moze raditi USB SEND / USB Keying(CW).
        # Ako su linije aktivne pri otvaranju, u CW modu radio odmah prelazi u TX.
        self.ser = None
        self._open_port()
        self.key_line = keyline        # 'RTS' ili 'DTR' - mora odgovarati postavci
                                       # USB Keying (CW) u radiju
        self._key_down = False
        self.cw_mode = 'buf'           # 'buf' = 17h buffer, 'key' = USB keying
        self.state = {}
        self.lock = threading.Lock()
        self.on_change = lambda d: None
        self.on_scope = lambda b: None
        self.sc = None
        self.scbuf = bytearray()
        self._reading_b = False     # sljedeci direktni 03 odgovor = TX (sub VFO B)
        self._direct = False        # je li okvir direktan odgovor (E0) ili broadcast (00)
        self._want_txfreq = False   # treba procitati TX (sub VFO B) - samo na dogadjaj
        self._cw_cancel = False     # prekid CW predaje (makro/direktni tekst)
        self._cw_wait_ack = False   # cekamo FB/NG potvrdu za 17h komad
        self._cw_ack = None         # 'ok' / 'ng' zadnja potvrda
        self._last_scope = 0.0      # vrijeme zadnjeg scope (27 00) okvira
        self.callsign = ''          # pozivni znak za zamjenu (call) u makroima

    def _open_port(self):
        """Otvori serijski port (tolerira ako radio nije spreman) i odmah
        ugasi DTR/RTS (sprijecava trenutni TX pri bootu zbog USB Keyinga)."""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception:
            self.ser = None
            return False
        try:
            self.ser.setDTR(False)
            self.ser.setRTS(False)
        except Exception:
            pass
        return True

    def _drop_port(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def send(self, cmd, data=b''):
        if not self.ser:
            return
        with self.lock:
            try:
                self.ser.write(bytes([0xFE, 0xFE, self.addr, CTRL]) + cmd + data + b'\xFD')
            except Exception:
                self._drop_port()

    def set(self, k, v):
        if self.state.get(k) != v:
            self.state[k] = v
            self.on_change({k: v})

    def key(self, down):
        """Spusta/podize key liniju (USB Keying CW)."""
        down = bool(down)
        if down == self._key_down:
            return
        self._key_down = down
        if not self.ser:
            return
        if self.key_line == 'DTR':
            self.ser.setDTR(down)
        else:
            self.ser.setRTS(down)

    def _precise_sleep(self, t):
        """Kratko spavanje s ms preciznoscu (hybrid: sleep + busy-wait)."""
        if t <= 0:
            return
        if t > 0.01:
            time.sleep(t - 0.004)
        end = time.perf_counter() + min(t, 0.01)
        while time.perf_counter() < end:
            pass

    def cw_key_send(self, text):
        """Predaja CW titranjem key linije (USB Keying CW) - bez buffera,
        prekidivo u bilo kojem trenutku, ravnomjerno na svim brzinama.
        U radiju: USB SEND = OFF, USB Keying (CW) = linija kao --keyline."""
        text = str(text).replace('(call)', self.callsign or '')
        data = ''.join(ch for ch in text.upper()
                       if ch in self.MORSE or ch == ' ')
        if not data.strip():
            return
        self._cw_cancel = False

        def worker():
            try:
                if self.state.get('mode') not in ('CW', 'CW-R'):
                    self.send(b'\x06', bytes([MODE_ID['CW'], self.state.get('filt', 1)]))
                    self.set('mode', 'CW')
                    time.sleep(0.1)
                word_gap = False
                for ch in data:
                    if self._cw_cancel:
                        break
                    if ch == ' ':
                        word_gap = True
                        continue
                    if word_gap:
                        u = 1.2 / max(5, self.state.get('cwspd') or 20)
                        self._precise_sleep(4 * u)     # dovrsetak word gapa (7u)
                        word_gap = False
                        if self._cw_cancel:
                            break
                    u = 1.2 / max(5, self.state.get('cwspd') or 20)
                    for i, el in enumerate(self.MORSE[ch]):
                        if self._cw_cancel:
                            break
                        self.key(True)
                        self._precise_sleep(u if el == '.' else 3 * u)
                        self.key(False)
                        if i < len(self.MORSE[ch]) - 1:
                            self._precise_sleep(u)     # unutar znaka
                    if self._cw_cancel:
                        break
                    self._precise_sleep(2 * u)         # razmak izmedu znakova
            finally:
                self.key(False)

        threading.Thread(target=worker, daemon=True).start()

    def cw_send(self, text):
        """Predaja CW teksta (17h, do 30 znakova po okviru). Ako nije CW mod,
        prebaci na CW (uz BK-IN u radiju radio odmah predaje). Buffer radija prima
        ogranicen broj znakova (~60), pa se svaki komad POTVRDUJE: NG odgovor =
        pun buffer -> komad se ponavlja nakon pauze. Prije sljedeceg komada ceka
        ~90% njegovog trajanja (racunato po brzini kljuca i razmacima), tako da
        buffer drži malo zalihe i u eteru NEMA pauza.
        Radi u pozadinskoj niti."""
        if self.cw_mode == 'key':
            self.cw_key_send(text)
            return
        text = str(text).replace('(call)', self.callsign or '')
        data = bytes(ord(ch) for ch in text if 0x20 <= ord(ch) <= 0x7E)
        if not data:
            return
        self._cw_cancel = False

        def worker():
            if self.state.get('mode') not in ('CW', 'CW-R'):
                self.send(b'\x06', bytes([MODE_ID['CW'], self.state.get('filt', 1)]))
                self.set('mode', 'CW')
                time.sleep(0.1)
            wpm = self.state.get('cwspd') or 20
            for i in range(0, len(data), 30):
                if self._cw_cancel:
                    break
                chunk = data[i:i + 30]
                while not self._cw_cancel:          # dok radio ne potvrdi (FB)
                    self._cw_ack = None
                    self._cw_wait_ack = True
                    self.send(b'\x17', chunk)
                    t0 = time.time()
                    while self._cw_ack is None and time.time() - t0 < 2.0:
                        time.sleep(0.02)
                    if self._cw_ack != 'ng':
                        break                       # 'ok' (ili nema odgovora) -> dalje
                    time.sleep(1.5)                 # pun buffer: pricekaj pa ponovi
                if self._cw_cancel:
                    break
                # Sljedeci komad saljemo kad je ~polovica prethodnog odigrana
                # (50% procjene: 6 jedinica/znak + 4/razmak). Buffer drži zalihu,
                # pa nikad ne prazni -> nema pauza ni ispadanja BKIN-a. Ako bi
                # pretjerali, radio odgovori NG i komad se ponovi (bez gubitka).
                time.sleep((6 * len(chunk) + 4 * chunk.count(b' ')) * 1.2
                           / max(5, wpm) * 0.5)

        threading.Thread(target=worker, daemon=True).start()

    def cw_stop(self):
        """Prekini CW predaju odmah: pusti key liniju, zaustavi nit, isprazni
        CW buffer radija (17h 00h) i vrati u RX (1Ch 00h 00h)."""
        self._cw_cancel = True
        try:
            self.key(False)
        except Exception:
            pass
        # 17 FF = zaustavlja slanje CW poruke (sluzbeni Icom kod! 17 00 ne radi)
        self.send(b'\x17', b'\xFF')
        self.send(b'\x1C\x00', b'\x00')   # odmah u RX
        self.set('ptt', 0)

    def reader(self):
        buf = bytearray()
        while True:
            try:
                if not self.ser:
                    # radio nedostupan (ugasen/resetiran) - pokusaj ponovno
                    if self._open_port():
                        print('** Serijski port otvoren:', self.port, '- ponovno očitavanje')
                        self.scope_init()
                        self.read_initial_state()
                    else:
                        print('!! Nema veze s radio-stanicom na', self.port,
                              '- pokusavam svake 2 s')
                    time.sleep(2.0)
                    continue
                buf += self.ser.read(64)
                while 0xFD in buf:
                    end = buf.index(0xFD)
                    frame = bytes(buf[:end]); del buf[:end + 1]
                    i = frame.find(b'\xFE\xFE')
                    if i < 0:
                        continue
                    f = frame[i + 2:]
                    # odgovor radija: to=E0 (ili 00 za transceive), from=addr
                    if len(f) >= 3 and f[1] == self.addr and f[0] in (CTRL, 0x00):
                        self._direct = (f[0] == CTRL)
                        try:
                            self.parse(f[2:])
                        except Exception as e:
                            print('parse err', e, f.hex())
            except Exception:
                # port je pao usred rada (npr. radio se restartao) - reconnect
                self._drop_port()
                time.sleep(2.0)

    def parse(self, p):
        c = p[0]
        if c in (0x00, 0x03) and len(p) >= 6:
            f = parse_freq(p[1:6])
            if c == 0x03 and self._reading_b and self._direct:
                self._reading_b = False
                self.set('txfreq', f)           # sub VFO (B) = TX u split modu
            else:
                self.set('freq', f)             # main VFO = RX
        elif c in (0x01, 0x04) and len(p) >= 2:
            self.set('mode', MODES.get(p[1], '?'))
            if len(p) >= 3:
                self.set('filt', p[2])          # filter bajt uz mod (FIL1/2/3 = 1/2/3)
        elif c == 0x0F and len(p) >= 2:
            self.set('split', int(p[1] == 1))
        elif c == 0x11 and len(p) >= 2:
            self.set('att', int(p[1] == 0x20))
        elif c == 0x14 and len(p) >= 4:
            for k, sub in LEVELS.items():
                if p[1] == sub:
                    if sub == 0x0C:   # key speed: 0000=6WPM ... 0255=48WPM
                        self.set(k, round(6 + parse_lvl(p[2:4]) * 42 / 255))
                    else:
                        self.set(k, parse_lvl(p[2:4]))
        elif c == 0x15 and len(p) >= 4 and p[1] in METERS:
            self.set(METERS[p[1]], parse_lvl(p[2:4]))
        elif c == 0x16 and len(p) >= 3:
            if p[1] == 0x02:
                self.set('preamp', p[2])
            for k, sub in FLAGS.items():
                if p[1] == sub:
                    self.set(k, p[2])
        elif c == 0x27 and len(p) >= 5 and p[1] == 0x00:      # scope (waterfall) podaci
            self._last_scope = time.time()
            seq, mx = int(f"{p[3]:02X}"), int(f"{p[4]:02X}")
            d = p[5:]
            if seq == 1 and len(d) >= 11:
                a, b = parse_freq(d[1:6]), parse_freq(d[6:11])
                self.sc = (max(0, a - b), a + b) if d[0] == 0 else (a, b)   # 0 = center mode
                self.scbuf = bytearray()
            elif seq > 1 and self.sc:
                self.scbuf += d
                if seq == mx:
                    # 0x01 = tip poruke "waterfall", zatim granice spektra, pa pikseli
                    self.on_scope(b'\x01' + struct.pack('>II', *self.sc) + bytes(self.scbuf))
        elif c == 0x1C and len(p) >= 3:
            if p[1] == 0x00:
                self.set('ptt', p[2])
            elif p[1] == 0x01:
                self.set('tuner', p[2])
        elif c in (0xFB, 0xFA):              # OK / NG potvrda naredbe
            if self._cw_wait_ack:
                self._cw_ack = 'ok' if c == 0xFB else 'ng'
                self._cw_wait_ack = False

    def apply(self, k, v):
        if k == 'freq':
            self.send(b'\x05', bcd_freq(v)); self.set('freq', int(v))
        elif k == 'mode' and v in MODE_ID:
            self.send(b'\x06', bytes([MODE_ID[v], 1])); self.set('mode', v)
        elif k == 'filter':                                   # FIL1/2/3 za trenutni mod
            m = MODE_ID.get(self.state.get('mode'))
            if m is not None:
                f = max(1, min(3, int(v)))
                self.send(b'\x06', bytes([m, f])); self.set('filt', f)
        elif k == 'vfo':
            self.send(b'\x07', {'A': b'\x00', 'B': b'\x01', 'X': b'\xB0', 'EQ': b'\xA0'}[v])
            self.send(b'\x03')
        elif k == 'cwspd':
            # IC-7300 CI-V: 14 0C -> 0000=6 WPM ... 0255=48 WPM (linearno!)
            wpm = max(6, min(48, int(v)))
            raw = int(round((wpm - 6) * 255 / 42))
            self.send(b'\x14\x0C', bcd_lvl(raw)); self.set('cwspd', wpm)
        elif k in LEVELS:
            self.send(b'\x14' + bytes([LEVELS[k]]), bcd_lvl(v)); self.set(k, int(v))
        elif k in FLAGS:
            self.send(b'\x16' + bytes([FLAGS[k]]), bytes([int(bool(v))])); self.set(k, int(bool(v)))
        elif k == 'preamp':
            self.send(b'\x16\x02', bytes([int(v)])); self.set('preamp', int(v))
        elif k == 'att':
            self.send(b'\x11', b'\x20' if v else b'\x00'); self.set('att', int(bool(v)))
        elif k == 'split':
            self.send(b'\x0F', bytes([int(bool(v))])); self.set('split', int(bool(v)))
        elif k == 'ptt':
            self.send(b'\x1C\x00', bytes([int(bool(v))])); self.set('ptt', int(bool(v)))
        elif k == 'span':
            self.send(b'\x27\x15\x00', bcd_freq(v))
        elif k == 'tuner':
            self.send(b'\x1C\x01', bytes([int(v)]))
        elif k == 'powerbtn':
            # 18 01 = power ON (radi samo iz standbyja, tj. ako je radio
            # iskljucen preko softvera), 18 00 = power OFF
            self.send(b'\x18', b'\x01' if v else b'\x00')
            if v:
                def _reinit():
                    time.sleep(4.0)     # pricekaj boot radija
                    self.scope_init()
                    time.sleep(6.0)     # i ponovno za svaki slucaj
                    self.scope_init()
                threading.Thread(target=_reinit, daemon=True).start()
        elif k == 'cwmode':
            self.cw_mode = 'key' if v == 'key' else 'buf'     # buffer ili USB keying
        elif k == 'cw':
            self.cw_send(str(v))                              # predaja CW teksta / makroa
        elif k == 'cw_stop':
            self.cw_stop()                                    # prekini makro predaju
        elif k == 'txfreq':
            self._want_txfreq = True                          # rucno osvjezi TX prikaz

    def scope_init(self):
        """Samo uključi CI-V izlaz scope podataka; ne mijenjaj LCD/SCOPE način."""
        self.send(b'\x27\x11\x01')
        self._last_scope = time.time()

    def read_initial_state(self):
        """Očitaj postavke radija bez mijenjanja njihovih vrijednosti."""
        commands = [b'\x03', b'\x04', b'\x0F', b'\x11',
                    b'\x1C\x00', b'\x1C\x01', b'\x15\x02']
        commands += [b'\x14' + bytes([sub]) for sub in LEVELS.values()]
        commands += [b'\x16' + bytes([sub]) for sub in (0x02, 0x22, 0x40, 0x41)]
        for cmd in commands:
            if not self.ser:
                break
            self.send(cmd)
            time.sleep(0.06)

    def poll(self):
        n = 0
        self.scope_init()
        self.read_initial_state()
        prev_split = prev_ptt = 0
        while True:
            split = 1 if self.state.get('split') else 0
            ptt = 1 if self.state.get('ptt') else 0
            if split and not prev_split:
                self._want_txfreq = True          # split upravo ukljucen
            if split and prev_ptt and not ptt:
                self._want_txfreq = True          # upravo zavrsio prijenos
            prev_split, prev_ptt = split, ptt

            if self._want_txfreq and split and not ptt:
                # TX = sub VFO (B). "Ples" VFO-om: odaberi B, procitaj, vrati na A.
                # Radi se SAMO na dogadjaj (ukljucen split / kraj TX / klik), pa se
                # prijem NE PREBACUJE s A - slusamo neprekidno na VFO A.
                self._want_txfreq = False
                self._reading_b = True
                self.send(b'\x07\x01'); time.sleep(0.05)
                self.send(b'\x03');    time.sleep(0.15)
                self.send(b'\x07\x00'); time.sleep(0.05)
                self.send(b'\x03')
            else:
                self.send(b'\x03')
            self.send(b'\x04'); self.send(b'\x15\x02')
            if self.ser and time.time() - self._last_scope > 8:   # scope stao? (power ciklus)
                self.scope_init()
            if self.state.get('ptt'):
                self.send(b'\x15\x11'); self.send(b'\x15\x12')
            elif self.state.get('pwr_meter') or self.state.get('swr'):
                self.set('pwr_meter', 0)   # reset u RX - ne ostaju zaleđene
                self.set('swr', 0)         # vrijednosti sa zadnjeg TX
            if n % 8 == 0:
                # Čitaj i aktivni filter: promjena na LCD-u mora se vidjeti na webu.
                self.send(b'\x04')
                for sub in LEVELS.values():
                    self.send(b'\x14' + bytes([sub]))
                for sub in (0x02, 0x22, 0x40, 0x41):
                    self.send(b'\x16' + bytes([sub]))
                for q in (b'\x11', b'\x0F', b'\x1C\x00', b'\x1C\x01'):
                    self.send(q)
            n += 1
            time.sleep(0.25)


def ulaw_enc(x):
    x = np.clip(x, -1, 1)
    y = np.sign(x) * np.log1p(255 * np.abs(x)) / np.log(256)
    return ((y + 1) / 2 * 255 + 0.5).astype(np.uint8)


def ulaw_dec(b):
    y = b.astype(np.float32) / 255 * 2 - 1
    return (np.sign(y) * (256 ** np.abs(y) - 1) / 255).astype(np.float32)


class Audio:
    """RX: USB codec -> 8 kHz u-law -> web. TX: web -> 8 kHz u-law -> USB codec."""
    def __init__(self, dev_in, dev_out, rate=8000, gain=3.0):
        self.dev_in, self.dev_out, self.k = dev_in, dev_out, 48000 // rate
        self.gain = float(gain)      # RX pojacanje prije slanja u browser (1 = bez)
        self.txbuf = np.zeros(0, np.float32)
        self.lock = threading.Lock()
        self.emit = lambda b: None

    def start(self, emit):
        self.emit = emit
        if self.dev_in is not None:
            self.inp = sd.InputStream(device=self.dev_in, samplerate=48000, channels=1,
                                      blocksize=1920, dtype='float32', callback=self.on_in)
            self.inp.start()
        if self.dev_out is not None:
            self.out = sd.OutputStream(device=self.dev_out, samplerate=48000, channels=1,
                                       blocksize=960, dtype='float32', callback=self.on_out)
            self.out.start()

    def on_in(self, data, frames, t, status):
        x = data[:, 0]
        # jednostavno decimiranje 48k -> 8k (prosjek grupa od 6 uzoraka)
        x = x[:len(x) // self.k * self.k].reshape(-1, self.k).mean(axis=1)
        self.emit(b'\x02' + ulaw_enc(x * self.gain).tobytes())

    def on_out(self, out, frames, t, status):
        with self.lock:
            n = min(frames, len(self.txbuf))
            out[:n, 0] = self.txbuf[:n]
            self.txbuf = self.txbuf[n:]
        out[n:] = 0

    def play(self, b):
        x = ulaw_dec(np.frombuffer(b, np.uint8))
        up = np.interp(np.arange(len(x) * self.k) / self.k, np.arange(len(x)), x).astype(np.float32)
        with self.lock:
            self.txbuf = np.concatenate([self.txbuf, up])[-24000:]


LOG_PAGE = '''<!doctype html><html lang="hr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>IC-7300 · QSO LOG</title>\n<style>:root{color-scheme:dark}*{box-sizing:border-box}body{font:14px system-ui,Arial;background:#10141a;color:#e6edf3;margin:0;padding:20px}.top{display:flex;gap:15px;align-items:center;flex-wrap:wrap;margin-bottom:18px}h2{margin:0;color:#f5a524}.panel{background:#1a212b;padding:16px;border-radius:12px}input{background:#10141a;border:1px solid #425064;border-radius:5px;color:#e6edf3;padding:7px;font:inherit}#search{min-width:240px}button{background:#2d5279;color:white;border:0;border-radius:6px;padding:8px 12px;cursor:pointer}button:hover{background:#386b9d}.tablewrap{overflow:auto;max-height:75vh}table{border-collapse:collapse;width:100%;white-space:nowrap}th{position:sticky;top:0;background:#273446;color:#f5a524;text-align:left;z-index:1}td,th{padding:9px 11px;border-bottom:1px solid #354050}tr:nth-child(even){background:#202b37}tr:hover{background:#304053}.edit{background:#b8791c;color:#fff}.delete{background:#9b3038;margin-left:8px}.delete:hover{background:#bb3c46}.modal{position:fixed;inset:0;background:#000a;display:none;align-items:center;justify-content:center;padding:16px}.dialog{background:#202b37;border:1px solid #516070;border-radius:12px;padding:22px;max-width:650px;width:100%;max-height:90vh;overflow:auto}.fields{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:12px;margin:18px 0}.fields label{display:flex;flex-direction:column;gap:5px;color:#b8c8d9}.fields input{width:100%}.actions{display:flex;justify-content:flex-end;gap:10px}#status{color:#8ccda1}</style></head><body>\n<div class="panel"><div class="top"><h2>IC-7300 · QSO LOG</h2><input id="search" placeholder="Search callsign, name, QTH…" oninput="render()"><button onclick="load()">Refresh</button><span id="count"></span><span id="status"></span></div><div class="tablewrap"><table><thead><tr><th>#</th><th>Date</th><th>UTC</th><th>Callsign</th><th>Band</th><th>MHz</th><th>Mode</th><th>RSTs</th><th>RSTr</th><th>Name</th><th>QTH</th><th></th></tr></thead><tbody id="rows"></tbody></table></div></div>\n<div class="modal" id="modal"><div class="dialog"><h2>Edit QSO</h2><div class="fields" id="fields"></div><div class="actions"><button onclick="closeEdit()">Odustani</button><button class="edit" id="save" onclick="saveEdit()">SAVE promjene</button></div><p id="error" style="color:#ff9d91"></p></div></div>\n<script>\nlet data=[], current=-1;const $=id=>document.getElementById(id);const cols=[\'QSO_DATE\',\'TIME_ON\',\'CALL\',\'BAND\',\'FREQ\',\'MODE\',\'RST_SENT\',\'RST_RCVD\',\'NAME\',\'QTH\'];const names=[\'Date YYYYMMDD\',\'UTC HHMMSS\',\'Callsign\',\'Band\',\'Frekvencija MHz\',\'Mode\',\'RSTsend\',\'RSTrecv\',\'Name\',\'QTH\'];const limits=[8,6,10,8,16,12,3,3,10,15];\nfunction td(tr,value){const cell=document.createElement(\'td\');cell.textContent=value||\'\';tr.appendChild(cell)}\nfunction render(){const q=$(\'search\').value.toLowerCase();const tbody=$(\'rows\');tbody.replaceChildren();let shown=0;data.forEach((r,i)=>{if(q&&!Object.values(r).some(v=>String(v).toLowerCase().includes(q)))return;shown++;const tr=document.createElement(\'tr\');td(tr,String(i+1));cols.forEach(k=>{let value=r[k]||'';if(k==='QSO_DATE'&&/^\\d{8}$/.test(value)){value=value.slice(6,8)+'/'+value.slice(4,6)+'/'+value.slice(0,4)}td(tr,value)});const cell=document.createElement(\'td\');const btn=document.createElement(\'button\');btn.className=\'edit\';btn.textContent=\'Edit\';btn.onclick=()=>edit(i);cell.appendChild(btn);const del=document.createElement('button');del.className='delete';del.textContent='Delete';del.onclick=()=>deleteQSO(i);cell.appendChild(del);tr.appendChild(cell);tbody.appendChild(tr)});$(\'count\').textContent=shown+\' / \'+data.length+\' QSO\'}\nasync function load(){try{const r=await fetch(\'/api/log/entries\',{cache:\'no-store\'});const d=await r.json();if(!r.ok||!d.ok)throw Error(d.err||\'Greška čitanja\');data=d.records;render();$(\'status\').textContent=\'\'}catch(e){$(\'status\').textContent=e.message}}\nfunction edit(i){current=i;$(\'fields\').replaceChildren();cols.forEach((k,j)=>{const label=document.createElement(\'label\');label.textContent=names[j];const inp=document.createElement(\'input\');inp.id=\'field_\'+k;inp.maxLength=limits[j];inp.value=data[i][k]||\'\';label.appendChild(inp);$(\'fields\').appendChild(label)});$(\'error\').textContent=\'\';$(\'modal\').style.display=\'flex\'}\nfunction closeEdit(){$(\'modal\').style.display=\'none\';current=-1}\nasync function deleteQSO(i){const r=data[i];if(!r)return;const label=(r.CALL||'?')+' / '+(r.QSO_DATE||'')+' '+(r.TIME_ON||'')+' UTC';if(!confirm('Trajno obrisati QSO '+label+'? Prije brisanja izrađuje se .adi.bak kopija.'))return;try{const res=await fetch('/api/log/entries/'+i,{method:'DELETE'});const d=await res.json();if(!res.ok||!d.ok)throw Error(d.err||'Greška brisanja');await load();$('status').textContent='QSO obrisan (backup .adi.bak)'}catch(e){$('status').textContent=e.message}}
async function saveEdit(){if(current<0)return;const fields={};cols.forEach(k=>fields[k]=$(\'field_\'+k).value);$(\'save\').disabled=true;try{const r=await fetch(\'/api/log/entries/\'+current,{method:\'PUT\',headers:{\'Content-Type\':\'application/json\'},body:JSON.stringify({fields})});const d=await r.json();if(!r.ok||!d.ok)throw Error(d.err||\'Greška spremanja\');closeEdit();await load();$(\'status\').textContent=\'Promjene spremljene u ADIF (backup .adi.bak)\'}catch(e){$(\'error\').textContent=e.message}finally{$(\'save\').disabled=false}}\nload();\n</script></body></html>'''

CW_PAGE = r"""<!DOCTYPE html>
<html lang="hr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>IC-7300 CW console</title>
<style>
:root{--bg:#0d1117;--panel:#161b22;--line:#2d3748;--txt:#dbe4ee;--dim:#7d8aa0;--grn:#3fb950;--red:#f85149}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:Segoe UI,Roboto,Arial,sans-serif;max-width:760px;margin:0 auto;padding:12px}
h1{font-size:18px;margin-bottom:12px;letter-spacing:1px}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px;margin-bottom:12px}
.rowlabel{color:var(--dim);font-size:12px;text-transform:uppercase;letter-spacing:1px;margin-bottom:6px}
.grid{display:flex;flex-wrap:wrap;gap:5px}
button{background:#1c2430;color:var(--txt);border:1px solid var(--line);border-radius:8px;padding:10px 14px;font-size:14px;cursor:pointer}
button:hover{border-color:#58a6ff}
button:active{transform:scale(.95)}
button.on{background:#123a24;border-color:var(--grn);color:var(--grn);font-weight:bold}
button.small{padding:6px 10px;font-size:12px}
button.warn{border-color:var(--red);color:#ffb3ae}
input[type=text]{width:100%;background:#0a0e14;border:1px solid var(--line);border-radius:8px;color:#f5a623;
 font-family:Consolas,monospace;font-size:18px;padding:10px 12px;letter-spacing:1px}
#macroinputs{display:grid;grid-template-columns:1fr;gap:5px;margin-top:8px}
#macroinputs input{background:#0a0e14;border:1px solid var(--line);border-radius:8px;color:var(--txt);
 padding:8px 10px;font-family:monospace;font-size:14px}
#stat{font-size:12px;color:var(--dim);margin-top:10px;font-family:monospace}
.row{display:flex;gap:6px;margin-top:8px}
.row input{flex:1}
</style>
</head>
<body>
<div style="display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:12px"><h1 style="margin-bottom:0">IC-7300 <span style="color:var(--dim)">CW CONSOLE</span></h1><button onclick="window.open('/','_blank')" style="background:#23517a;font-weight:700">Save QSO</button></div>

<div class="panel">
  <div class="rowlabel">Filter</div>
  <div class="grid" id="filts"></div>
</div>

<div class="panel">
  <div class="rowlabel">CW makronaredbe (klik = predaja)</div>
  <div class="row" style="align-items:center"><span style="font-size:12px;color:var(--dim)">Callsign:</span><input id="callsign" placeholder="Call" maxlength="12" style="flex:0 0 130px;padding:5px 8px;border-radius:8px;border:1px solid var(--line);background:#0a0e14;color:#f5a623;font-family:monospace;font-size:13px"><span style="font-size:12px;color:var(--dim);margin-left:8px">RST:</span><input id="rst" placeholder="599" maxlength="6" style="flex:0 0 70px;padding:5px 8px;border-radius:8px;border:1px solid var(--line);background:#0a0e14;color:#f5a623;font-family:monospace;font-size:13px"></div>
  <div class="grid" id="macros"></div>
  <div class="row">
    <button class="small" onclick="toggleEdit()">Edit macros</button>
    <button class="small warn" onclick="send('cw_stop')">Stop CW / TX</button>
  </div>
  <div id="macroedit" style="display:none">
    <div id="macroinputs"></div>
    <button class="small" style="margin-top:6px" onclick="saveMacros()">Save macros</button>
  </div>
</div>

<div class="panel">
  <div class="rowlabel">Direct CW text</div>
  <div class="row">
    <input type="text" id="cwtext" placeholder="Direct CW text input... K" autocomplete="off">
    <button onclick="sendText()">TX</button>
  </div>
</div>

<div id="stat">connecting...</div>

<script>
const $ = id => document.getElementById(id);
let ws = null, MACROS = ['','','','','','','','','','','','','','',''];

function send(k, v = null){ if(ws && ws.readyState === 1) ws.send(JSON.stringify({k, v})); }

// --- filteri ---
[1,2,3].forEach(f => {
  const b = document.createElement('button');
  b.textContent = 'FIL' + f; b.id = 'f' + f; b.style.flex = '1';
  b.onclick = () => send('filter', f);
  $('filts').appendChild(b);
});

// --- makroi ---
let MNAMES = ['M1','M2','M3','M4','M5','M6','M7','M8','M9','M10','M11','M12','M13','M14','M15'];
function cwSub(t){ return t.replace(/\(call\)/g, ($('callsign').value || '').trim().toUpperCase())
                         .replace(/\(rst\)/g, (($('rst') || {value:''}).value || '').trim().toUpperCase()); }
function build(){
  const box = $('macros'); box.innerHTML = '';
  MACROS.forEach((t, i) => {
    const b = document.createElement('button');
    b.className = 'small'; b.style.flex = '1';
    b.textContent = MNAMES[i] + (t ? ' ·' : '');
    b.title = MNAMES[i] + ': ' + (t || '(prazno)');
    b.onclick = () => { if (t) send('cw', cwSub(t)); };
    b.oncontextmenu = ev => { ev.preventDefault();
      const n = (prompt('New macro name ' + MNAMES[i] + ' (max 8 char):', MNAMES[i]) || '').trim().slice(0, 8);
      if (n) { MNAMES[i] = n; saveMacros(); } };
    box.appendChild(b);
  });
}
function toggleEdit(){
  const ed = $('macroedit');
  ed.style.display = ed.style.display === 'none' ? 'block' : 'none';
  const box = $('macroinputs'); box.innerHTML = '';
  MACROS.forEach((t, i) => {
    const inp = document.createElement('input');
    inp.value = t; inp.maxLength = 120;
    inp.placeholder = 'M' + (i + 1) + ' (max 120 characters)';
    inp.oninput = () => MACROS[i] = inp.value;
    box.appendChild(inp);
  });
}
function saveMacros(){
  fetch('/api/macros', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({macros: MACROS, names: MNAMES})})
    .then(() => { $('macroedit').style.display = 'none'; build(); });
}
fetch('/api/macros').then(r => r.json()).then(d => {
  MACROS = (d.macros || []).concat(['','','','','','','','','','','','','','','']).slice(0,15);
  if (d.names) MNAMES = (d.names || []).concat(['M1','M2','M3','M4','M5','M6','M7','M8','M9','M10','M11','M12','M13','M14','M15']).slice(0,15);
  build(); });

// --- direktni tekst ---
function sendText(){
  const el = $('cwtext');
  if (el.value.trim()) send('cw', cwSub(el.value));
  el.value = ''; el.focus();
}
$('cwtext').addEventListener('keydown', e => { if (e.key === 'Enter') sendText(); });

// --- websocket: stanje radija (isto kao index.html) ---
function connect(){
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(proto + '://' + location.host + '/ws');
  ws.onopen = () => $('stat').textContent = 'LINK OK';
  ws.onclose = () => { $('stat').textContent = 'LINK disconnected - connecting...'; setTimeout(connect, 2000); };
  ws.onmessage = e => {
    if (typeof e.data !== 'string') return;              // binarno: waterfall/audio
    try {
      const d = JSON.parse(e.data);
      if (d.filt !== undefined) [1,2,3].forEach(f => $('f' + f).classList.toggle('on', d.filt === f));
      $('stat').textContent = 'LINK OK · mod ' + (d.mode !== undefined ? d.mode : '')
        + ' · FIL' + (d.filt !== undefined ? d.filt : '')
        + (d.ptt ? ' · TX' : '');
    } catch (x) {}
  };
}
connect();
</script>
</body>
</html>
"""


INDEX_HTML = r"""<!doctype html>
<html lang="hr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>IC-7300 Web</title>
<style>
:root{--bg:#10141a;--card:#1a212b;--fg:#e6edf3;--mut:#7d8a99;--acc:#f5a524;--on:#2ea043;--tx:#e5484d}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif;padding:8px;max-width:1000px;margin:auto}
.card{background:var(--card);border-radius:8px;padding:8px;margin-bottom:6px}
header{display:flex;justify-content:space-between;color:var(--mut);font-size:13px;margin-bottom:4px}
#freq{font:600 clamp(22px,5.5vw,38px) ui-monospace,Consolas,monospace;color:var(--acc);text-align:center;user-select:none;letter-spacing:1px}
#freq span{cursor:ns-resize;padding:0 1px}#freq span:hover{background:#ffffff14;border-radius:4px}
#freq .dot{cursor:default;padding:0}
.row{display:flex;flex-wrap:wrap;gap:4px;margin:2px 0}
button{background:#2a3441;color:var(--fg);border:0;border-radius:5px;padding:4px 8px;font-size:12.5px;cursor:pointer}
button:hover{background:#344255}button.on{background:var(--on)}button.sel{background:var(--acc);color:#000}
#ptt{flex:1;padding:10px;font-size:16px;font-weight:700;touch-action:none}#ptt.on{background:var(--tx)}
.sl{display:grid;grid-template-columns:56px 1fr 40px;gap:6px;align-items:center;margin:3px 0;font-size:12.5px}
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:16px;background:transparent;cursor:pointer}
input[type=range]::-webkit-slider-runnable-track{height:6px;border-radius:3px;background:linear-gradient(90deg,var(--on) var(--fill,50%),#3a4553 var(--fill,50%))}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;appearance:none;width:14px;height:14px;margin-top:-4px;border-radius:50%;background:#e6edf3;border:2px solid var(--on)}
input[type=range]::-moz-range-track{height:6px;border-radius:3px;background:#3a4553}
input[type=range]::-moz-range-progress{height:6px;border-radius:3px;background:var(--on)}
input[type=range]::-moz-range-thumb{width:12px;height:12px;border-radius:50%;background:#e6edf3;border:2px solid var(--on)}
.meter{height:10px;background:#0b0f14;border-radius:4px;overflow:hidden}.meter div{height:100%;background:linear-gradient(90deg,#2ea043,#f5a524 60%,#e5484d);width:0}
.grid{display:grid;grid-template-columns:1.15fr 1fr 1.15fr;gap:6px}@media(max-width:760px){.grid{grid-template-columns:1fr}}
h4{margin:0 0 4px;color:var(--mut);font-weight:500;font-size:11px;text-transform:uppercase}
canvas{display:block;background:#0b0f14}
</style></head><body>
<header><b>IC-7300 web remote server by 9a3zi - ICweb v2.01</b><span><span id="splitb" style="color:#e5484d;font-weight:700;display:none">SPLIT · </span><span id="st">○ connecting…</span></span></header>

<div class="card"><div id="freq"></div>
<div id="txf" style="text-align:center;color:#e5484d;font:700 15px ui-monospace,monospace;display:none;margin-top:-2px;cursor:pointer" title="Klik = ponovno ocitaj TX frekvenciju"></div>
<div class="row"><input id="fin" placeholder="MHz, ex. 14.074" style="flex:1;padding:5px 8px;border-radius:5px;border:0;background:#0b0f14;color:var(--fg);font-size:13px">
<button onclick="goto()"> >>> </button>
<button onclick="tx('vfo','A')">A</button><button onclick="tx('vfo','B')">B</button>
<button onclick="tx('vfo','X')">A⇄B</button><button onclick="tx('vfo','EQ')">A=B</button></div>
<div class="row" id="bands"></div></div>

<div class="card"><canvas id="cv" width="475" height="120" style="width:100%;border-radius:6px;cursor:crosshair"></canvas>
<div class="row" id="spans" style="align-items:center"></div>
<div class="row" style="align-items:center;color:var(--mut)">Down <input type="range" id="flo" min="0" max="120" value="20" style="width:90px"> Up <input type="range" id="fhi" min="40" max="160" value="120" style="width:90px"></div></div>

<div class="card" style="padding:6px 8px"><div class="row" style="align-items:center;margin:0"><button id="aud">🔇 Audio</button>
<span style="font-size:12px">Volume</span><input type="range" id="vol" min="0" max="200" value="100" style="width:110px"><span id="mic" style="color:var(--mut);font-size:11px"></span></div></div>

<div class="grid">
<div class="card"><h4>Mode</h4><div class="row" id="modes"></div>
<h4 style="margin-top:10px">Filter</h4><div class="row" id="filts"></div>
<h4 style="margin-top:10px">Functions</h4><div class="row" id="tog"></div>
<div class="row" id="pre"></div></div>

<div class="card"><h4>Meters</h4>
<div class="sl"><span>S</span><div class="meter"><div id="m_s"></div></div><span id="t_s"></span></div>
<div class="sl"><span>PWR</span><div class="meter"><div id="m_p"></div></div><span id="t_p"></span></div>
<div class="sl"><span>SWR</span><div class="meter"><div id="m_w"></div></div><span id="t_w"></span></div></div>

<div class="card" id="sliders"></div>
</div>

<div class="card"><h4></h4>
<div class="row" style="align-items:center"><span style="font-size:12px;color:var(--mut)">Callsign:</span><input id="callsign" placeholder="Call" maxlength="10" autocomplete="off" style="flex:0 0 10ch;width:10ch;min-width:0;padding:5px 3px;border-radius:5px;border:0;background:#0b0f14;color:#f5a623;font-family:ui-monospace,monospace;font-size:13px"><span style="font-size:12px;color:var(--mut);margin-left:8px">RSTsend→:</span><input id="rst" placeholder="599" maxlength="3" autocomplete="off" title="Poslani raport (makroi (rst) povlače ovaj)" style="flex:0 0 3.8ch;width:3.8ch;min-width:0;padding:5px 2px;border-radius:5px;border:0;background:#0b0f14;color:#f5a623;font-family:ui-monospace,monospace;font-size:13px"><span style="font-size:12px;color:var(--mut);margin-left:6px">RSTrecv←:</span><input id="rstrcv" placeholder="599" maxlength="3" autocomplete="off" title="Primljeni raport" style="flex:0 0 3.8ch;width:3.8ch;min-width:0;padding:5px 2px;border-radius:5px;border:0;background:#0b0f14;color:#f5a623;font-family:ui-monospace,monospace;font-size:13px"><span style="font-size:12px;color:var(--mut);margin-left:6px">Name:</span><input id="qsoname" maxlength="10" autocomplete="off" style="flex:0 0 105px;min-width:0;padding:5px 8px;border-radius:5px;border:0;background:#0b0f14;color:#f5a623;font-family:ui-monospace,monospace;font-size:13px"><span style="font-size:12px;color:var(--mut);margin-left:6px">QTH:</span><input id="qth" maxlength="15" autocomplete="off" style="flex:0 0 145px;min-width:0;padding:5px 8px;border-radius:5px;border:0;background:#0b0f14;color:#f5a623;font-family:ui-monospace,monospace;font-size:13px"><button id="savebtn" onclick="saveQSO()">SAVE</button><button onclick="window.open('/log','_blank')" style="margin-left:22px;background:#23517a;font-weight:700">LOG</button><button onclick="window.open('/cw','_blank')" style="background:#23517a;font-weight:700">CW console</button></div>
<div class="row" style="align-items:center"><span style="font-size:12px;color:var(--mut)">My call:</span><input id="mycall" placeholder="Call" maxlength="12" autocomplete="off" onchange="saveMy()" style="flex:0 0 110px;padding:5px 8px;border-radius:5px;border:0;background:#0b0f14;color:#f5a623;font-family:ui-monospace,monospace;font-size:13px"><span style="font-size:12px;color:var(--mut);margin-left:8px">My loc:</span><input id="grid" placeholder="JN85" maxlength="8" autocomplete="off" onchange="saveMy()" style="flex:0 0 70px;padding:5px 8px;border-radius:5px;border:0;background:#0b0f14;color:#f5a623;font-family:ui-monospace,monospace;font-size:13px"></div>
<div class="row" id="macros"></div>
<div class="row"><button onclick="toggleEdit()">Edit macro</button>
<button id="cwmodebtn" onclick="toggleCWMode()">CW: buffer</button>
</div>
<div id="medit" style="display:none">
<div id="minp" style="display:grid;gap:4px;margin:6px 0"></div>
<button onclick="saveMacros()">Save macros</button></div>
<h4 style="margin-top:10px"></h4>
<div class="row"><input id="cwtxt" placeholder="Direct cw text input ..." autocomplete="off"
 style="flex:1;padding:5px 8px;border-radius:5px;border:0;background:#0b0f14;color:var(--fg);font-family:ui-monospace,monospace;font-size:13px">
<button onclick="sendCW()">TX</button></div>
</div>
</div>

<div class="row"><button id="ptt">PTT (hold)</button><button onclick="tx('tuner',2)">TUNE</button>
<button style="background:var(--tx);font-weight:700" onclick="tx('cw_stop')">STOP TX</button>
<button onclick="if(confirm('Isključiti IC-7300?'))tx('powerbtn',0)">POWER OFF</button>
<button onclick="tx('powerbtn',1)" title="Radi samo ako je radio u standbyu (isključen preko softvera, ne na tipku)">POWER ON</button></div>

<script>
const S={};let ws;const $=id=>document.getElementById(id);
const BANDS=[['160',1840000],['80',3700000],['60',5357000],['40',7100000],['30',10120000],['20',14200000],['17',18100000],['15',21200000],['12',24900000],['10',28400000],['6',50150000],['4',70100000]];
const MODES=['LSB','USB','CW','CW-R','AM','FM','RTTY'];
const TOG=[['nb','NB'],['nr','NR'],['anf','ANF'],['att','ATT 20dB'],['split','SPLIT'],['tuner','TUNER']];
const PCT=['af','rf','sql','power'];   // ovi se prikazuju 0-100 (radi vrijednost / 2.55)
const SL=[['af','AF',0,255,0],['rf','RF',0,255,255],['sql','SQL',0,255,0],['power','TX power',0,255,0],['cwspd','CW speed',4,48,20]];
const PRE=[[0,'PRE OFF'],[1,'P.AMP1'],[2,'P.AMP2']];
const tx=(k,v=null)=>ws&&ws.readyState==1&&ws.send(JSON.stringify({k,v}));
function setFill(el){const mn=+el.min||0,mx=+el.max||100;
  el.style.setProperty('--fill',((el.value-mn)/(mx-mn)*100)+'%')}
addEventListener('input',e=>{if(e.target.type==='range')setFill(e.target)});

// frekvencija: klik gore/dolje ili kotačić na znamenci
const PL=[1e7,1e6,1e5,1e4,1e3,100,10,1];
PL.forEach((p,i)=>{
  const s=document.createElement('span');s.dataset.p=p;s.id='d'+i;$('freq').appendChild(s);
  if(i==1||i==4){const d=document.createElement('span');d.className='dot';d.textContent='.';$('freq').appendChild(d)}
  const step=dir=>tx('freq',Math.max(30000,(S.freq||0)+dir*p));
  s.onclick=e=>step(e.offsetY<s.offsetHeight/2?1:-1);
  s.onwheel=e=>{e.preventDefault();step(e.deltaY<0?1:-1)};
});
function goto(){const v=parseFloat($('fin').value.replace(',','.'));if(v>0)tx('freq',Math.round(v*1e6))}
$('fin').onkeydown=e=>{if(e.key=='Enter')goto()};

// UI elementi
for(const[b,f]of BANDS){const e=document.createElement('button');e.textContent=b+'m';e.onclick=()=>tx('freq',f);$('bands').appendChild(e)}
MODES.forEach(m=>{const e=document.createElement('button');e.textContent=m;e.id='mode_'+m;e.onclick=()=>tx('mode',m);$('modes').appendChild(e)});
TOG.forEach(([k,l])=>{const e=document.createElement('button');e.textContent=l;e.id='tog_'+k;e.onclick=()=>tx(k,S[k]?0:1);$('tog').appendChild(e)});
PRE.forEach(([v,l])=>{const e=document.createElement('button');e.textContent=l;e.id='pre_'+v;e.onclick=()=>tx('preamp',v);$('pre').appendChild(e)});
SL.forEach(([k,l,mn,mx,df])=>{$('sliders').insertAdjacentHTML('beforeend',`<div class="sl"><span>${l}</span><input type="range" id="s_${k}" min="${mn}" max="${mx}" value="${df}"><span id="v_${k}"></span></div>`);
  $('s_'+k).oninput=e=>{$('v_'+k).textContent=PCT.includes(k)?Math.round(e.target.value/2.55):e.target.value;tx(k,+e.target.value)}});
document.querySelectorAll('input[type=range]').forEach(setFill);   // pocetna ispuna prema value atributu

// filteri FIL1/2/3
[1,2,3].forEach(f=>{const e=document.createElement('button');e.textContent='FIL'+f;e.id='filt_'+f;
  e.onclick=()=>tx('filter',f);$('filts').appendChild(e)});

// CW makronaredbe M1-M8 + direktni unos
let MACROS=['','','','','','','','','','','','','','',''],MNAMES=['M1','M2','M3','M4','M5','M6','M7','M8','M9','M10','M11','M12','M13','M14','M15'];
let CWKEY=localStorage.getItem('cwkey')==='1';
if(CWKEY){$('cwmodebtn').textContent='CW: KEYING';$('cwmodebtn').className='on'}
function toggleCWMode(){CWKEY=!CWKEY;localStorage.setItem('cwkey',CWKEY?'1':'0');
  $('cwmodebtn').textContent=CWKEY?'CW: KEYING':'CW: buffer';
  $('cwmodebtn').className=CWKEY?'on':'';tx('cwmode',CWKEY?'key':'buf')}
function cwSub(t){return t.replace(/\(call\)/g,($('callsign').value||'').trim().toUpperCase())
  .replace(/\(rst\)/g,($('rst').value||'').trim().toUpperCase())}
function buildMacros(){const box=$('macros');box.innerHTML='';
  MACROS.forEach((t,i)=>{const e=document.createElement('button');e.textContent=MNAMES[i]+(t?' ·':'');
    e.title=MNAMES[i]+': '+(t||'(empty)')+'  (right click = rename)';
    e.onclick=()=>t&&tx('cw',cwSub(t));
    e.oncontextmenu=ev=>{ev.preventDefault();renameMacro(i)};box.appendChild(e)})}
function renameMacro(i){
  const n=(prompt('New macro name '+MNAMES[i]+' (max 8 char):',MNAMES[i])||'').trim().slice(0,8);
  if(!n)return;
  MNAMES[i]=n;saveMacros();
}
function toggleEdit(){const ed=$('medit');ed.style.display=ed.style.display=='none'?'block':'none';
  const box=$('minp');box.innerHTML='';
  MACROS.forEach((t,i)=>{const e=document.createElement('input');e.value=t;e.maxLength=120;
    e.placeholder=MNAMES[i]+' (max 120 characters)';
    e.style.cssText='padding:8px;border-radius:6px;border:0;background:#0b0f14;color:var(--fg);font-family:ui-monospace,monospace';
    e.oninput=()=>MACROS[i]=e.value;box.appendChild(e)})}
function saveMy(){fetch('/api/macros',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({macros:MACROS,names:MNAMES,mycall:($('mycall').value||'').trim().toUpperCase(),
    gridsquare:($('grid').value||'').trim().toUpperCase()})})}
function saveMacros(){fetch('/api/macros',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({macros:MACROS,names:MNAMES,mycall:($('mycall').value||'').trim().toUpperCase(),
    gridsquare:($('grid').value||'').trim().toUpperCase()})})
  .then(()=>{$('medit').style.display='none';buildMacros()})}
function sendCW(){const e=$('cwtxt');if(e.value.trim())tx('cw',cwSub(e.value));e.value='';e.focus()}
function saveQSO(){
  const call=($('callsign').value||'').trim().toUpperCase(),rst=($('rst').value||'').trim().toUpperCase()||'599',
        rstr=($('rstrcv').value||'').trim().toUpperCase()||'599',
        name=($('qsoname').value||'').trim(),qth=($('qth').value||'').trim();
  if(!call){const b=$('savebtn');b.textContent='UPISI CALL!';setTimeout(()=>b.textContent='SAVE',1500);return}
  fetch('/api/log',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({call,rst,rstr,name,qth})}).then(r=>r.json()).then(d=>{
      const b=$('savebtn');b.textContent=d.ok?('OK QSO '+d.count):'GRESKA';
      if(d.ok){ ['callsign','rst','rstrcv','qsoname','qth'].forEach(id=>$(id).value=''); $('callsign').focus(); }
      setTimeout(()=>b.textContent='SAVE',1500)})}
// TAB navigacija isključivo kroz QSO unos i SAVE (Shift+Tab ide unatrag).
const QSO_TAB_IDS=['callsign','rst','rstrcv','qsoname','qth','savebtn'];
document.addEventListener('keydown', e=>{
  if(e.key!=='Tab'||e.altKey||e.ctrlKey||e.metaKey)return;
  // Ne presreći Tab dok je otvoren dijalog ili drugi prozor za uređivanje.
  const current=QSO_TAB_IDS.indexOf(document.activeElement.id);
  if(current<0 && document.activeElement!==document.body && document.activeElement!==document.documentElement)return;
  e.preventDefault();
  const next=current<0?0:(current+(e.shiftKey?-1:1)+QSO_TAB_IDS.length)%QSO_TAB_IDS.length;
  $(QSO_TAB_IDS[next]).focus();
});
$('cwtxt').onkeydown=e=>{if(e.key=='Enter')sendCW()};
fetch('/api/macros').then(r=>r.json()).then(d=>{
  MACROS=(d.macros||[]).concat(['','','','','','','','','','','','','','','']).slice(0,15);
  if(d.names)MNAMES=(d.names||[]).concat(['M1','M2','M3','M4','M5','M6','M7','M8','M9','M10','M11','M12','M13','M14','M15']).slice(0,15);
  if(d.mycall)$('mycall').value=d.mycall;
  if(d.gridsquare)$('grid').value=d.gridsquare;
  buildMacros()});

// PTT: drži za TX (i touch)
const ptt=$('ptt');
['pointerdown'].forEach(n=>ptt.addEventListener(n,e=>{ptt.setPointerCapture(e.pointerId);tx('ptt',1)}));
['pointerup','pointercancel'].forEach(n=>ptt.addEventListener(n,()=>tx('ptt',0)));
addEventListener('blur',()=>S.ptt&&tx('ptt',0));

$('txf').onclick=()=>tx('txfreq',1);
function fmtT(hz){return Math.floor(hz/1e6)+'.'+String(Math.floor(hz%1e6/1e3)).padStart(3,'0')+'.'+String(Math.floor(hz%1e3/10)).padStart(2,'0')}
function sTxt(v){if(v<=120)return'S'+Math.round(v/120*9);return'S9+'+Math.round((v-120)/121*60)+'dB'}
function render(){
  if(S.freq!=null){const s=String(S.freq).padStart(8,'0');PL.forEach((p,i)=>$('d'+i).textContent=s[i])}
  MODES.forEach(m=>$('mode_'+m).className=S.mode==m?'sel':'');
  [1,2,3].forEach(f=>$('filt_'+f).className=S.filt===f?'sel':'');
  TOG.forEach(([k])=>$('tog_'+k).className=S[k]?'on':'');
  PRE.forEach(([v])=>$('pre_'+v).className=S.preamp==v?'sel':'');
  SL.forEach(([k])=>{if(S[k]!=null&&document.activeElement!==$('s_'+k)){$('s_'+k).value=S[k];setFill($('s_'+k));$('v_'+k).textContent=PCT.includes(k)?Math.round(S[k]/2.55):S[k]}});
  ptt.className=S.ptt?'on':'';
  if(S.split&&S.txfreq!=null){$('txf').style.display='block';$('txf').textContent='TX '+fmtT(S.txfreq)}
  else $('txf').style.display='none';
  $('splitb').style.display=S.split?'inline':'none';
  if(S.smeter!=null){$('m_s').style.width=S.smeter/241*100+'%';$('t_s').textContent=sTxt(S.smeter)}
  if(S.pwr_meter!=null){$('m_p').style.width=S.pwr_meter/255*100+'%';$('t_p').textContent=Math.round(S.pwr_meter/2.55)+'%'}
  if(S.swr!=null){$('m_w').style.width=S.swr/255*100+'%';$('t_w').textContent=S.swr<48?(1+S.swr/48*0.5).toFixed(1):S.swr<80?'~2':'>3'}
}
function conn(){
  ws=new WebSocket((location.protocol=='https:'?'wss':'ws')+'://'+location.host+'/ws');
  ws.onopen=()=>{ws.binaryType='arraybuffer';$('st').textContent='● connected';
    tx('cwmode',CWKEY?'key':'buf')};
  ws.onclose=()=>{$('st').textContent='○ disconnected';setTimeout(conn,1500)};
  ws.onmessage=e=>{if(typeof e.data!=='string')return onBin(e.data);Object.assign(S,JSON.parse(e.data));render()};
}

// ---------- waterfall ----------
const cv=$('cv'),cx=cv.getContext('2d'),W=475,H=120,SH=40;let sc={lo:0,hi:0};
const STOPS=[[0,0,0],[0,0,140],[0,200,220],[255,230,0],[255,40,0]];
const LUT=Array.from({length:256},(_,i)=>{const x=i/255*4,j=Math.min(3,x|0),f=x-j;return STOPS[j].map((a,k)=>a+(STOPS[j+1][k]-a)*f)});
cx.fillStyle='#0b0f14';cx.fillRect(0,0,W,H);
function drawScope(d){
  const n=d.length,lo=+$('flo').value,hi=Math.max(lo+10,+$('fhi').value);
  cx.drawImage(cv,0,SH,W,H-SH-1,0,SH+1,W,H-SH-1);
  const img=cx.createImageData(W,1);
  for(let x=0;x<W;x++){const v=d[Math.min(n-1,x*n/W|0)],t=Math.max(0,Math.min(1,(v-lo)/(hi-lo))),c=LUT[t*255|0];img.data.set([c[0],c[1],c[2],255],x*4)}
  cx.putImageData(img,0,SH);
  cx.fillStyle='#0b0f14';cx.fillRect(0,0,W,SH);cx.strokeStyle='#f5a524';cx.beginPath();
  for(let x=0;x<W;x++){const y=SH-1-d[Math.min(n-1,x*n/W|0)]/160*(SH-2);x?cx.lineTo(x,y):cx.moveTo(x,y)}
  cx.stroke();
  if(S.freq&&sc.hi>sc.lo){const x=(S.freq-sc.lo)/(sc.hi-sc.lo)*W;cx.fillStyle='#e5484d';cx.fillRect(x|0,0,1,H)}
}
cv.onclick=e=>{if(sc.hi<=sc.lo)return;const r=cv.getBoundingClientRect();
  tx('freq',Math.round((sc.lo+(e.clientX-r.left)/r.width*(sc.hi-sc.lo))/10)*10)};
[2500,5000,10000,25000,50000,100000,250000,500000].forEach(v=>{const b=document.createElement('button');
  b.textContent='±'+(v>=1000?v/1000+'k':v);b.onclick=()=>tx('span',v);$('spans').appendChild(b)});

// ---------- audio ----------
let ac,gain,nextT=0;
function onBin(buf){const v=new DataView(buf),t=v.getUint8(0);
  if(t==1){sc.lo=v.getUint32(1);sc.hi=v.getUint32(5);drawScope(new Uint8Array(buf,9))}
  else if(t==2)playRx(new Uint8Array(buf,1))}
function playRx(d){
  if(!ac||S.ptt)return;
  const f=new Float32Array(d.length);
  for(let i=0;i<d.length;i++){const y=d[i]/255*2-1;f[i]=Math.sign(y)*(Math.pow(256,Math.abs(y))-1)/255}
  const b=ac.createBuffer(1,f.length,8000);b.copyToChannel(f,0);
  const s=ac.createBufferSource();s.buffer=b;s.connect(gain);
  const t=ac.currentTime;if(nextT<t+0.03)nextT=t+0.15;s.start(nextT);nextT+=b.duration}
$('vol').oninput=()=>gain&&(gain.gain.value=$('vol').value/100);
$('aud').onclick=async()=>{
  if(ac)return;
  ac=new AudioContext();gain=ac.createGain();gain.gain.value=$('vol').value/100;gain.connect(ac.destination);
  $('aud').className='on';$('aud').textContent='🔊 Audio ON';
  try{
    const st=await navigator.mediaDevices.getUserMedia({audio:{echoCancellation:false,noiseSuppression:false,autoGainControl:false}});
    const src=ac.createMediaStreamSource(st),mp=ac.createScriptProcessor(2048,1,1),r=ac.sampleRate/8000;
    mp.onaudioprocess=e=>{
      if(!S.ptt||!ws||ws.readyState!=1)return;
      const x=e.inputBuffer.getChannelData(0),n=Math.floor(x.length/r),o=new Uint8Array(n+1);o[0]=3;
      for(let i=0;i<n;i++){const a=Math.floor(i*r),b=Math.floor((i+1)*r);let s=0;for(let j=a;j<b;j++)s+=x[j];s/=(b-a);
        const m=Math.sign(s)*Math.log1p(255*Math.min(1,Math.abs(s)))/Math.log(256);o[i+1]=Math.round((m+1)/2*255)}
      ws.send(o)};
    src.connect(mp);mp.connect(ac.destination);$('mic').textContent='🎤 microphone ready';
  }catch(e){$('mic').textContent='mikrofon nedostupan (treba HTTPS ili localhost)'}
};
conn();
</script></body></html>
"""


def make_app(radio, password, audio=None):
    app = web.Application(client_max_size=1024 * 1024)
    app['clients'] = set()
    app['macros'], app['macnames'], app['mycall'], app['grid'] = load_macros()
    root = Path(__file__).parent

    @web.middleware
    async def auth(req, handler):
        if password:
            h = req.headers.get('Authorization', '')
            ok = False
            if h.startswith('Basic '):
                try:
                    ok = base64.b64decode(h[6:]).decode().split(':', 1)[1] == password
                except Exception:
                    pass
            if not ok:
                return web.Response(status=401, headers={'WWW-Authenticate': 'Basic realm="IC-7300"'})
        return await handler(req)
    app.middlewares.append(auth)

    async def index(_):
        return web.Response(text=INDEX_HTML, content_type='text/html', charset='utf-8')

    async def cw_page(_):
        return web.Response(text=CW_PAGE, content_type='text/html', charset='utf-8')

    async def api_ping(_):
        return web.json_response({'ok': True, 'radio_connected': bool(radio.ser)})

    async def api_macros_get(_):
        return web.json_response({'macros': app['macros'], 'names': app['macnames'],
                                  'mycall': app['mycall'], 'gridsquare': app['grid']})

    async def api_macros_post(req):
        try:
            d = await req.json()
            app['macros'] = norm8(d.get('macros', []))
            app['macnames'] = norm_names(d.get('names', app['macnames']))
            app['mycall'] = str(d.get('mycall', app['mycall']))[:12]
            app['grid'] = str(d.get('gridsquare', app['grid']))[:8]
            save_macros(app['macros'], app['macnames'], app['mycall'], app['grid'])
        except Exception:
            return web.json_response({'ok': False}, status=400)
        return web.json_response({'ok': True})

    async def ws_handler(req):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(req)
        app['clients'].add(ws)
        print('** WS klijent spojen:', req.remote)
        try:
            await ws.send_str(json.dumps(radio.state))
            async for m in ws:
                try:
                    if m.type == web.WSMsgType.TEXT:
                        d = json.loads(m.data)
                        k = d.get('k')
                        if k:
                            radio.apply(k, d.get('v'))
                    elif m.type == web.WSMsgType.BINARY and audio and m.data[:1] == b'\x03' \
                            and radio.state.get('ptt'):
                        audio.play(m.data[1:])
                except Exception as e:
                    print('cmd err', repr(e))
        except Exception:
            import traceback
            print('!! WS PUKAO - razlog:')
            traceback.print_exc()
        finally:
            app['clients'].discard(ws)
            print('** WS klijent odspojen')
            if not app['clients'] and radio.state.get('ptt'):   # failsafe: nitko ne drzi PTT
                radio.apply('ptt', 0)
        return ws

    async def on_start(app):
        loop = asyncio.get_running_loop()

        async def bcast(d):
            for c in list(app['clients']):
                try:
                    await c.send_str(json.dumps(d))
                except Exception:
                    pass
        radio.on_change = lambda d: loop.call_soon_threadsafe(asyncio.ensure_future, bcast(d))

        async def bcast_bin(b):
            for c in list(app['clients']):
                try:
                    await c.send_bytes(b)
                except Exception:
                    pass
        post = lambda b: app['clients'] and loop.call_soon_threadsafe(asyncio.ensure_future, bcast_bin(b))
        radio.on_scope = post
        if audio:
            audio.start(post)
        threading.Thread(target=radio.reader, daemon=True).start()
        if hasattr(radio, 'poll'):                            # poll nije bitan za rad
            threading.Thread(target=radio.poll, daemon=True).start()

    app.on_startup.append(on_start)
    app.router.add_get('/', index)
    app.router.add_get('/cw', cw_page)
    app.router.add_get('/api/ping', api_ping)
    app.router.add_get('/api/macros', api_macros_get)
    app.router.add_post('/api/macros', api_macros_post)
    app.router.add_get('/ws', ws_handler)
    async def api_log(req):
        try:
            d = await req.json()
        except Exception:
            return web.json_response({'ok': False, 'err': 'los zahtjev'}, status=400)
        call = str(d.get('call', '')).strip().upper()[:10]
        rst = str(d.get('rst', '')).strip().upper()[:3] or '599'
        rstr = str(d.get('rstr', '')).strip().upper()[:3] or '599'   # zadani primljeni raport
        name = str(d.get('name', '')).strip()[:80]
        qth = str(d.get('qth', '')).strip()[:120]
        if not call:
            return web.json_response({'ok': False, 'err': 'nema pozivnog znaka'}, status=400)
        freq = int(radio.state.get('freq') or 0)
        mode = str(radio.state.get('mode') or '')
        count = adif_append(call, rst, rstr, freq, mode, app['mycall'], app['grid'], name, qth)
        return web.json_response({'ok': True, 'count': count})

    async def log_page(req):
        return web.Response(text=LOG_PAGE, content_type='text/html')

    async def log_entries(req):
        try:
            with ADIF_LOCK:
                _, records = adif_load()
            return web.json_response({'ok': True, 'records': [
                {k: adif_field(rec, k) for k in EDIT_FIELDS} for rec in records]})
        except Exception as e:
            return web.json_response({'ok': False, 'err': str(e)}, status=500)

    async def log_update(req):
        try:
            idx = int(req.match_info['index'])
            d = await req.json()
            changes = d.get('fields', {})
            if not isinstance(changes, dict): raise ValueError('Neispravni podaci')
            adif_update(idx, changes)
            return web.json_response({'ok': True})
        except (ValueError, TypeError) as e:
            return web.json_response({'ok': False, 'err': str(e)}, status=400)
        except Exception as e:
            return web.json_response({'ok': False, 'err': str(e)}, status=500)

    async def log_delete(req):
        try:
            idx = int(req.match_info['index'])
            adif_delete(idx)
            return web.json_response({'ok': True})
        except (ValueError, TypeError) as e:
            return web.json_response({'ok': False, 'err': str(e)}, status=400)
        except Exception as e:
            return web.json_response({'ok': False, 'err': str(e)}, status=500)

    app.router.add_get('/log', log_page)
    app.router.add_get('/api/log/entries', log_entries)
    app.router.add_put('/api/log/entries/{index}', log_update)
    app.router.add_delete('/api/log/entries/{index}', log_delete)
    app.router.add_post('/api/log', api_log)
    return app


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--port',  default=None, help='COM3 (Silicon Labs CP210x Enhanced COM)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--addr', type=lambda x: int(x, 16), default=0x94, help='CI-V adresa (hex), IC-7300 = 94')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--web-port', type=int, default=8080)
    ap.add_argument('--password', default='')
    ap.add_argument('--audio-in', default=None, help='indeks ili ime USB Audio CODEC ulaza (RX, zvuk iz radija)')
    ap.add_argument('--audio-out', default=None, help='indeks ili ime USB Audio CODEC izlaza (TX)')
    ap.add_argument('--gain', type=float, default=3.0, help='RX pojacanje (1 = bez pojacanja, 3 = 3x glasnije)')
    ap.add_argument('--keyline', default='RTS', choices=['RTS', 'DTR'],
                    help='serijska linija za USB Keying (CW) - mora odgovarati meniju radija')
    ap.add_argument('--list-audio', action='store_true', help='ispisi audio uredjaje')
    ap.add_argument('--cert', help='cert.pem za HTTPS (potreban za mikrofon izvan localhosta)')
    ap.add_argument('--key', help='key.pem')
    a = ap.parse_args()
    if a.list_audio:
        if sd is None:
            ap.error('pip install numpy sounddevice')
        print(sd.query_devices()); raise SystemExit
    if not a.port:
        ap.error('--port je obavezan')
    dev = lambda s: int(s) if s is not None and s.isdigit() else s
    audio = Audio(dev(a.audio_in), dev(a.audio_out), gain=a.gain) if (a.audio_in or a.audio_out) and np else None
    if (a.audio_in or a.audio_out) and not np:
        print('UPOZORENJE: numpy/sounddevice nisu instalirani, audio je iskljucen')
    ctx = None
    if a.cert:
        ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ctx.load_cert_chain(a.cert, a.key)
    print('=' * 60)
    print('  IC-7300 WEB REMOTE', VERSION)
    print('  glavno sucelje:  http://localhost:%d/' % a.web_port)
    print('  CW console:      http://localhost:%d/cw' % a.web_port)
    print('=' * 60)
    web.run_app(make_app(Radio(a.port, a.baud, a.addr, a.keyline), a.password, audio),
                host=a.host, port=a.web_port, ssl_context=ctx)
