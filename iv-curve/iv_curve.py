#!/usr/bin/env python3
"""
2025-06-30

pip
PyMeasure-0.15.0-py3-none-any.whl.metadata

u24 apt
Version: 0.9.0-2
"""

from pymeasure.instruments.keithley import Keithley2400
from pymeasure.adapters import PrologixAdapter
from pymeasure.adapters import SerialAdapter
import glob
import time
import matplotlib.pyplot as plt
import numpy as np
try:
    from gxs700.util import hexdump
except ImportError:
    hexdump = None
import serial
import math

SER_BEGIN = "\x13"
SER_END = "\x11"

class DmmError(Exception):
    pass


class DmmSystemError(DmmError):
    pass


class DmmTimeout(DmmError):
    pass


class DMMParseError(DmmError):
    pass

# TODO: try higher performance version that returns immediately on END
class ZSerialAdapter(SerialAdapter):
    def __init__(self, *args, **kwargs):
        SerialAdapter.__init__(self, *args, **kwargs)

    def ask(self, command, timeout=None, wait=True):
        if timeout is None:
            timeout = 1.0
        """
        Fixes 2 issues:
        -\r needed to complete command
        -DC control characters at beginning and end screw up eventual split()

        Other: think serial is reading to timeout
        Really we should read DC begin and DC end to know transmission begin / end
        """

        # GPIB adapters have implicit end of command
        # Without this newline serial adapters don't work correctly
        # print("ask out: '%s'" % command)
        command = command + "\r"
        0 and hexdump(command.encode("ASCII"), "tx")
        ret = SerialAdapter.ask(self, command)
        tstart = time.time()
        # 00000000  13 30 2C 22 4E 6F 20 65  72 72 6F 72 22 11 0A     |.0,"No error".. |
        0 and hexdump(ret.encode("ASCII"), "rx")
        # ret = ret.strip()

        if wait:
            while True:
                """
                K2700 / K2750, K2000: think has SER_BEGIN/SER_END
                K2401 w/ ser: no termination
                Are these instrument settings I can tweak?
                """
                if SER_END in ret or "\r" in ret:
                    break
                if time.time() - tstart > timeout:
                    raise DmmTimeout("Timeout")
                try:
                    ret += self.read()
                except UnicodeDecodeError:
                    print("decode error, have", ret)
                    pass
                0 and hexdump(ret.encode("ASCII"), "retry")

        # DC control characters are really messing up split()
        if len(ret) and ret[0] == SER_BEGIN:
            ret = ret[1:]
        if len(ret) and ret[-1] == SER_END:
            ret = ret[0:-1]
        return ret.strip()

    def write(self, command):
        command = command + "\r"
        0 and hexdump(command.encode("ASCII"), "tx")
        return SerialAdapter.write(self, command)


class TestK2401Ser:
    def __init__(self, port=None):
        if port is None:
            # usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0
            devices = glob.glob("/dev/serial/by-id/usb-Prolific_*")
            assert len(devices), "No serial found"
            port = devices[0]
        # print("using", port)



        # self.adapter = ZSerialAdapter(port, 57600, timeout=0)
        port = serial.Serial(port,
                                timeout=0.001,
                                baudrate=57600,
                                writeTimeout=0)
        self.adapter = ZSerialAdapter(port=port)

        # print(type(self.adapter))
        # print(dir(self.adapter))
        self.instrument = Keithley2400(self.adapter)
        self.verbose = 0

        # KEITHLEY INSTRUMENTS INC.,MODEL 2401,4072483,A01 Aug 25 2011 12:57:43/A02  /T/K
        print("IDN test")
        print("got", self.cmd("*IDN?"))
        self.clear_errors()

        if 0:
            self.test1()

    def ident(self):
        # just vendor, model
        return self.ident_ex()[0:2]

    def ident_ex(self):
        '''
        Returns the manufacturer, model number, serial
        number, and firmware revision levels of the
        unit.
        ['KEITHLEY INSTRUMENTS INC.', 'MODEL 2750', '0967413', 'A07  /A01']
        '''
        tmp = self.ask("*IDN?").strip()
        # hack
        # AssertionError: ("'\x13KEITHLEY INSTRUMENTS INC.'",)
        if tmp[0] == "\x13":
            tmp = tmp[1:]
        # print("ident debug", tmp)
        ret = tmp.split(',')
        self.vendor = ret[0]
        self.model = ret[1]
        sn = ret[2]
        fw = ret[3]
        return (self.vendor, self.model, sn, fw)


    def cmd(self, q):
        if self.verbose:
            print("")
            print(q.strip())
        self.adapter.write(q + "\n")
        self.adapter.connection.flush()
        # 0.15 unreliable, 0.2 whole, add margin
        time.sleep(0.2)
        s = self.adapter.read().strip()
        self.verbose and print("  ", s)
        self.check_error()
        return s

    def check_error(self):
        self.adapter.write(":system:error?\n")
        self.adapter.connection.flush()
        time.sleep(0.2)
        s = self.adapter.read().strip()
        if s != '0,"No error"':
            print(s)

    def clear_errors(self):
        while True:
            try:
                self.check_error()
                break
            except DmmSystemError:
                pass

    def parse_vir(self, s):
        # sometimes 3, sometimes 5 parts
        # usually 5
        # V, I, R, ? , ?
        # +6.486189E-01,+4.560000E-03,+9.910000E+37,+8.039714E+03,+3.482000E+04
        try:
            v, i, r = s.strip().split(",")[0:3]
        except:
            # bad +1.500000E+00,+1.542443E-06,+9.91000
            print("bad", s)
            raise
        v = float(v)
        i = float(i)
        r = float(r)
        return v, i, r

    def meas_v(self, retries=3):
        for tryi in range(3):
            try:
                return self.parse_vir(self.cmd(":MEASure:VOLT?"))[0]
            except Exception as e:
                print("WARNING: error", e)
                if tryi == 2:
                    raise

    def meas_i(self, retries=3):
        for tryi in range(3):
            try:
                return self.parse_vir(self.cmd(":MEASure:CURR?"))[1]
            except Exception as e:
                print("WARNING: error", e)
                if tryi == 2:
                    raise

    def meas_r(self, retries=3):
        for tryi in range(3):
            try:
                return self.parse_vir(self.cmd(":MEASure:RES?"))[2]
            except Exception as e:
                print("WARNING: error", e)
                if tryi == 2:
                    raise

    def pv_iv(self):
        vmax = 2.4
        imax = 0.1
        steps = 10

        print("init...")
        self.cmd(":SYSTem:BEEPer:STATe 0")
        # Disable auto ranging since we are setting IV
        # self.cmd(":SENS:RES:MODE MAN")
        # Disable enable interlock
        self.cmd(":OUTP:ENAB OFF")
        self.cmd(":SOUR:FUNC:MODE VOLT")
        self.cmd(":CURRENT:PROT:LEV %f" % imax)
        f = open("out.csv", "w")
        f.write("v,i,r\n")
        f.flush()
        m_vs = []
        m_is = []
        m_rs = []

        # :SENSe:CURRent:RANGe:AUTO ON
        # :SENSe:VOLTage:RANGe:AUTO ON
        # :SOURce:CURRent:RANGe:AUTO ON
        # self.cmd(":SOURce:CURRent:RANGe:AUTO ON")
        self.cmd(":SOURce:VOLTage:RANGe:AUTO ON")

        # for step in range(steps + 1):
        #    iset = imax * step / steps
        #for step, vset in enumerate([0.1, 0.5, 1.0, 1.5, 2.0, 2.5]):
        # for step, vset in enumerate([0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]):
        try:
            # for step, vset in enumerate([0.1, 0.2, 0.3, 0.4, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]):
            # for step, vset in enumerate([-10, -9, -8, -7, -6.0, -5.5, -5.0, -4.5, -4.0, -3.5, -3.0, -2.5, -2.0, -1.5, -1.0, -0.5, 0.0, +0.5, +1.0, +1.5, +2.0, +2.5, +3.0, +3.5, +4.0, +4.5, +5.0, +5.5, +6.0, 7, 8, 9, 10]):
            for step, vset in enumerate([-5.0, -4.5, -4.0, -3.5, -3.0, -2.5, -2.0, -1.5, -1.0, -0.5, 0.0, +0.5, +1.0, +1.5, +2.0, +2.5, +3.0, +3.5, +4.0, +4.5, +5.0]):
            # for step, vset in enumerate([-5.0]):
                # :SOUR:CURR:LEV:IMM:AMPL 0.001
                self.cmd(":SOUR:VOLT:LEV:IMM:AMPL %f" % vset)
                if step == 0:
                    self.cmd(":OUTPUT ON")
                    # getting weird initial reading
                    # maybe wait a bit to stabalize?
                    time.sleep(1)
                    self.cmd(":SOUR:VOLT:LEV:IMM:AMPL %f" % vset)
                time.sleep(1)
                print("%f" % vset)
                
                v = np.median([self.meas_v() for x in range(3)])
                i = np.median([self.meas_i() for x in range(3)])
                r = np.median([self.meas_r() for x in range(3)])

                m_vs.append(v)
                m_is.append(i)
                m_rs.append(r)
                print("  V: %f" % v)
                print("  I: %f" % i)
                print("  R: %f" % r)
                f.write("%s,%s,%s\n" % (v, i, r))
                f.flush()
        finally:
            self.cmd(":OUTPUT OFF")

        plt.plot(m_vs, [x * 1e3 for x in m_is])
        plt.ylabel("Current (mA)")
        plt.xlabel("Voltage (V)")
        plt.savefig("plot.png")
        #plt.show()

def main():
    dmm = TestK2401Ser()
    dmm.pv_iv()


if __name__ == "__main__":
    main()
