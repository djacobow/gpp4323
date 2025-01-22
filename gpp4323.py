#!/usr/bin/env python3

import socket, serial, datetime, functools, threading, re, argparse, sys

def dprint(*args, **kwargs):
    print(datetime.datetime.now().isoformat(timespec='seconds') + ': ', end='')
    print(*args, **kwargs)
    sys.stdout.flush()

def _locked_gpp(method):
    def inner(ref, *args, **kwargs):
        with ref.gpp.lock:
            return method(ref, *args, **kwargs)
    return inner

class Monitor:
    @staticmethod
    def ABOVE(n):
        return ('>', n)
    
    @staticmethod
    def BELOW(n):
        return ('<', n)
    
    @staticmethod
    def EQUAL(n):
        return ('=', n)

    NONE = None
    
    TRIG_NONE = 0
    TRIG_OUTOFF = 1
    TRIG_ALARM = 2
    TRIG_BEEPER = 4

class Channel:
    def __init__(self, gpp, n):
        self.gpp = gpp
        self.n = n
    
    def meas(self):
        return self.gpp.meas().asDict()[self.n]
    
    @_locked_gpp
    def disable(self):
        self.gpp._sendline(f":OUTP{self.n}:STAT OFF")
        self.gpp.wait()
    
    @_locked_gpp
    def enable(self):
        self.gpp._sendline(f":OUTP{self.n}:STAT ON")
        self.gpp.wait()
    
    # returns the configured state
    @_locked_gpp
    def status(self):
        rv = {}
        vars_of_interest = (
            (":MODE{n}?",      r'(\S+)',      'mode',    lambda x: x[1]),
            (":SOUR{n}:CURR?", r'([\d\.]+)',  'current', lambda x: float(x[1])),
            (":SOUR{n}:VOLT?", r'([\d\.]+)',  'voltage', lambda x: float(x[1])),
            (":LOAD{n}:CV?",   r'(\S+)',      'load_cv', lambda x: x[1] == 'ON'),
            (":LOAD{n}:CC?",   r'(\S+)',      'load_cc', lambda x: x[1] == 'ON'),
            (":LOAD{n}:CR?",   r'(\S+)',      'load_cr', lambda x: x[1] == 'ON'),
            (":OUTP{n}:STAT?", r'(\S+)',      'enabled', lambda x: x[1] == 'ON'),
        )
        for v in vars_of_interest:
            self.gpp._sendline(v[0].format(**self.__dict__))
            rv[v[2]] = v[3](self.gpp._expect(v[1]))

        return rv                    

    # TODO: OVP/OCP
    @_locked_gpp
    def set_source(self, voltage, current):
        self.gpp._sendline("TRACK0")
        if self.n == 1 or self.n == 2:
            self.gpp._sendline(f":LOAD{self.n}:CC OFF")
        self.gpp._sendline(f":SOUR{self.n}:CURR {current}")
        self.gpp._sendline(f":SOUR{self.n}:VOLT {voltage}")
        self.gpp.wait()
        if self.n == 1 or self.n == 2:
            self.gpp._sendline(f":MODE{self.n}?")
            rv = self.gpp._expect(r'(\S+)')
            if rv.group(1) != 'IND':
                raise RuntimeError('supply failed to switch to source mode')
    
    @_locked_gpp
    def set_load(self, cv = None, cc = None, cr = None):
        if (cv is None and cc is None and cr is None) or \
           (cv is not None and cc is not None) or \
           (cv is not None and cr is not None) or \
           (cc is not None and cr is not None):
            raise ValueError(f'supply can track only one of CV / CC / CR mode at a time {cc} {cv} {cr}')
        if cv is not None:
            mode = 'CV'
            self.gpp._sendline(f":SOUR{self.n}:VOLT {cv}")
            self.gpp._sendline(f":LOAD{self.n}:CV on")
        elif cc is not None:
            mode = 'CC'
            self.gpp._sendline(f":SOUR{self.n}:CURR {cc}")
            self.gpp._sendline(f":LOAD{self.n}:CC on")
        elif cr is not None:
            mode = 'CR'
            self.gpp._sendline(f":LOAD{self.n}:RES {cr}")
            self.gpp._sendline(f":LOAD{self.n}:CR on")
        self.gpp.wait()
        self.gpp._sendline(f":MODE{self.n}?")
        rv = self.gpp._expect(r'(\S+)')
        if rv.group(1) != mode:
            raise RuntimeError(f'supply failed to switch to {mode} mode')
    
    @_locked_gpp
    def is_load(self):
        self.gpp._sendline(f":MODE{self.n}?")
        rv = self.gpp._expect(r'(\S+)')
        return rv.group(1) == 'CV' or rv.group(1) == 'CC' or rv.group(1) == 'CR'
    
    @_locked_gpp
    def monitor(self, current = Monitor.NONE, voltage = Monitor.NONE, power = Monitor.NONE, trigger = 0):
        self.gpp._sendline(f":MONI{self.n}:STAT OFF")

        # set, then clear all -- at least one must be set at all times,
        # apparently
        if current:
            self.gpp._sendline(f":MONI{self.n}:CURR:COND {current[0]}C,AND")
            self.gpp._sendline(f":MONI{self.n}:CURR:VAL {current[1]}")
        if voltage:
            self.gpp._sendline(f":MONI{self.n}:VOLT:COND {voltage[0]}V,AND")
            self.gpp._sendline(f":MONI{self.n}:VOLT:VAL {voltage[1]}")
        if power:
            self.gpp._sendline(f":MONI{self.n}:POWER:COND {power[0]}P")
            self.gpp._sendline(f":MONI{self.n}:POWER:VAL {power[1]}")

        if not current:
            self.gpp._sendline(f":MONI{self.n}:CURR:COND NONE,NONE")
        if not voltage:
            self.gpp._sendline(f":MONI{self.n}:VOLT:COND NONE,NONE")
        if not power:
            self.gpp._sendline(f":MONI{self.n}:POWER:COND NONE")

        # set, then clear all -- at least one must be set at all times,
        # apparently
        self.gpp._sendline(f":MONI{self.n}:STOP BEEPER,ON")
        self.gpp._sendline(f":MONI{self.n}:STOP OUTOFF,{'ON' if trigger & Monitor.TRIG_OUTOFF else 'OFF'}")
        self.gpp._sendline(f":MONI{self.n}:STOP ALARM,{'ON' if trigger & Monitor.TRIG_ALARM  else 'OFF'}")
        self.gpp._sendline(f":MONI{self.n}:STOP BEEPER,{'ON' if trigger & Monitor.TRIG_BEEPER else 'OFF'}")
        self.gpp.wait()
        if trigger and (current or voltage or power):
            self.gpp._sendline(f":MONI{self.n}:STAT ON")
        self.gpp.wait()
        
        # back to the home screen
        self.gpp._sendline(':DISP:TYPE 1')
    
    @_locked_gpp
    def sequence(self, seq, active = True):
        self.gpp._sendline(f':SEQU{self.n}:STAT OFF')
        self.gpp._sendline(f':SEQU{self.n}:GROUP {len(seq.groups)}')
        for n,grp in enumerate(seq.groups):
            self.gpp._sendline(f':SEQU{self.n}:PARA {n},{grp[0]},{grp[1]},{grp[2]}')
        self.gpp._sendline(f':SEQU{self.n}:STAR {seq.start}')
        if seq.cycles == True:
            self.gpp._sendline(f':SEQU{self.n}:CYCLE I')
        else:
            self.gpp._sendline(f':SEQU{self.n}:CYCLE N,{seq.cycles}')
        self.gpp._sendline(f':SEQU{self.n}:ENDS {seq.end}')
        self.gpp.wait()
        if active:
            self.gpp._sendline(f':SEQU{self.n}:STAT ON')
            self.gpp.wait()

        # back to the home screen
        self.gpp._sendline(':DISP:TYPE 1')
    
    def sequence_enable(self, active = True):
        self.gpp._sendline(f':SEQU{self.n}:STAT {"ON" if active else "OFF"}')

class Measurement:
    def __init__(self, parent):
        self.gpp = parent
        self.update()
    
    def __repr__(self):
        return '\n'.join([
            f"[Ch#{self.data[d]['channel']}: {self.data[d]['voltage']}V, {self.data[d]['current']}A, {self.data[d]['power']}W mode:{self.data[d]['mode']}]" for d in self.data
        ])

    @_locked_gpp
    def update(self):
        self.gpp._sendline(":MEAS?")
        rv = self.gpp._expect(r'([0-9.,;]+)')
        chs = [ch.split(',') for ch in rv.group(1).split(';')]
        self.data = {chn+1: {
            'voltage': float(ch[0]),
            'current': float(ch[1]),
            'power'  : float(ch[2]),
            'channel': chn+1 } for chn,ch in enumerate(chs)
        }
        for chn in self.data:
            self.gpp._sendline(f":MODE{chn}?")
            rv = self.gpp._expect(r'(\S+)')
            self.data[chn]['mode'] = rv[1]

    def asDict(self):
        return self.data


class GPPException(Exception):
        def __init__(self, message):
            super().__init__(message)

class GPP4323:
    def __init__(self, host=None, sport=None, debug=False):
        self.lock = threading.RLock()
        self.debug         = debug
        self.s = None
        if host is not None:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            dprint(f'Trying to connect to (host,port): {host}')
            self.s.connect(host)
            self.file = self.s.makefile('rwb')
        elif sport is not None and sport[0] is not None and sport[1] is not None:
            dprint(f'Trying to connect to (port,speed): {sport}')
            self.s = serial.Serial(sport[0], sport[1], timeout=2)
            self.file = self.s
        else:
            raise GPPException('You need to instantiate with host or serial port info; a tuple of host,port or serial_port,speed')

        try:
            self._sendline('*IDN?')
            rv = self._expect(r'([^,]*),([^,]*),SN:([^,]*),([^,]*)')
            self.manufacturer = rv.group(1)
            self.model = rv.group(2)
            self.serial = rv.group(3)
            self.version = rv.group(4).strip()
            self._sendline('*CLS')
            self._sendline(':SYST:CLE')
            self._sendline(':STAT:QUE:ENAB (-440:+900)')
        except Exception as e:
            raise GPPException(repr(e))

        dprint(f"Connected to {self.manufacturer} {self.model}, serial number {self.serial}, FW version {self.version}")
        
        # read the error queue: :STAT:QUE?
    
    def __del__(self):
        if self.s is not None:
            self.s.close()
    
    def wait(self):
        with self.lock:
            self._sendline("*OPC")
            tries = 5
            while tries > 0:
                self._sendline("*ESR?")
                val = int(self._expect(r'(\d+)').group(1))
                if val & 128:
                    dprint(f"NOTE: SN{self.serial} reports power cycle")
                if val & 32:
                    raise RuntimeError('command syntax error from supply')
                if val & 16:
                    raise RuntimeError('execution error from supply')
                if val & 8:
                    raise RuntimeError('device error from supply')
                if val & 4:
                    dprint(f"NOTE: SN{self.serial} reports query error")
                if val & 1:
                    break
                tries -= 1
            if tries == 0:
                raise RuntimeError('supply never became ready')

    @functools.cache
    def channel(self, n):
        return Channel(self, n)

    def meas(self):
        return Measurement(self)

    def alloff(self):
        self._sendline("ALLOUTOFF")

    def _expect(self, pattern):
        s = self.file.readline()
        if self.debug:
            dprint('<== ', s)
        s = s.decode('utf-8')
        m = re.search(pattern,s)
        return m

    def _sendline(self, l):
        if self.debug:
            dprint('==> ', l)
        self.file.write((l + '\n').encode('ascii'))
        self.file.flush()


class CliApp():
    def getArgs(self):
        p = argparse.ArgumentParser(description="library and cli tool for remote control of GPP4323 PSU")
        mug0 = p.add_mutually_exclusive_group()
        mug0.add_argument(
            '--host',
            default='192.168.1.72',
            help='hostname or IP',
        )
        mug0.add_argument(
            '--serial', '-s',
            default=None,
            help='Serial port'
        )
    
        mug1 = p.add_mutually_exclusive_group()
        mug1.add_argument(
            '--port', '-p',
            default=1026,
            type=int,
            help='host port',
        )
        mug1.add_argument(
            '--speed',
            default=115200,
            help='Serial port speed'
        )
    
        p.add_argument(
            '--debug',
            action='store_true',
            help='show debug strings to and from instrument'
        )
    
        p.add_argument(
            '--channel', '-c',
            default=1,
            help='channel number to control',
            type=int,
            choices=(1,2,3,4),
        )
    
    
        s = p.add_subparsers(help='commands', dest='command')
        s.add_parser('meas', help='report the current readings of all channels')
        s.add_parser('stat', help='report the current status of all channels')
    
        source_p = s.add_parser('source', help='source mode with max voltage and current')
        source_p.add_argument('--voltage', '-v', type=float, required=True)
        source_p.add_argument('--current', '-i', type=float, required=True)
    
        load_p = s.add_parser('load', help='load mode as either constant voltage, current, or resistance') 
        lp_mug0 = load_p.add_mutually_exclusive_group()
        lp_mug0.add_argument('--voltage', '-v', type=float)
        lp_mug0.add_argument('--current', '-i', type=float)
        lp_mug0.add_argument('--resistance', '-r', type=float)
    
        s.add_parser('enable', help='enable specified channel')
        s.add_parser('disable', help='disable specified channel')
     
        return p.parse_args()
    
    def showMeas(self, m):
        dprint('Measurement:')
        for ch, chv in m.asDict().items():
            dprint('  chan [{channel}] {mode:4} {voltage:7.3f}V {current:7.3f}A {power:7.3f}W'.format(**chv))
    
    def showStat(self, g, chans=(1,2,3,4)):
        dprint('Status:')
        s = {}
        for ch in chans:
            s[ch] = g.channel(ch).status()
        for ch, chv in s.items():
            load_mode_str = 'source'
            for load_mode in ('cv', 'cc', 'cr'):
                if chv.get('load_' + load_mode, False):
                    load_mode_str = load_mode
                    break
            en_str = 'enabled' if chv.get('enabled',False) else 'disabled'
            dprint('  chan [{channel}] {mode:4} {voltage:7.3f}V {current:7.3f}A {load_mode}, {en_str}'.format(channel=ch, **chv, load_mode=load_mode_str, en_str=en_str))
    
    def __init__(self):
        a = self.getArgs()
        if a.host is not None:
            self.g = GPP4323(host=(a.host, a.port), debug=a.debug)
        elif a.serial is not None:
            self.g = GPP4323(sport=(a.serial, a.speed), debug=a.debug)
        else:
            dprint('Error; no machine specified')
            sys.exit(-1)

        self.a = a

    def runit(self):
        a = self.a
        match a.command:
            case 'meas':
                self.showMeas(self.g.meas())
            case 'stat':
                self.showStat(self.g)
            case 'source':
                self.g.channel(a.channel).set_source(a.voltage, a.current)
                self.showStat(self.g, (a.channel,))
            case 'load':    
                self.g.channel(a.channel).set_load(cv = a.voltage, cc = a.current, cr = a.resistance)
                self.showStat(self.g, (a.channel,))
            case 'enable':
                self.g.channel(a.channel).enable()
                self.showStat(self.g, (a.channel,))
            case 'disable':
                self.g.channel(a.channel).disable()
                self.showStat(self.g, (a.channel,))
            case _:
                self.showStat(self.g)
                self.showMeas(self.g.meas())
    

if __name__ == '__main__':
    CliApp().runit()

