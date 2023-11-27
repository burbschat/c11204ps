"""
Python Code for Power Supply Hamamatsu c11204-01/02
---------------------------------------------------
Can be used to control Hamamatsu c11204 power supply via python 3


Copyright (C) 2019  Malinda de Silva

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>

"""

import serial
import serial.tools.list_ports
import binascii
import numpy as np


class CLAWSps:
    def __init__(self):
        self.ser = 0
        "Initialise the c11204 power supply. Make sure to connect power supply to the PC before initializing"
        # VARIABLES
        self.V_conversion = 1.812 * 10 ** (-3)  # voltage conversion factor
        self.I_conversion = 4.980 * 10 ** (-3)  # current conversion factor (mA)

        # FIXED VALUES
        self.STX = "02"  # start of text
        self.ETX = "03"  # end of text
        self.CR = "0D"  # delimiter

        # USER DEFINED VARIABLES
        self.V_lim_upper = 60  # Upper high voltage limit in Volts
        """ NOTE - The applied voltage can be set upto 90 V by c11204 power supply.
                   Change the upper voltage limit (self.V_lim_upper) as required by the MPPC in use"""

        # OPEN SERIAL PORT
        ports = list(serial.tools.list_ports.comports())
        if len(ports) == 0:
            self.logger.warning("Could not find any serial device!")
        for p in ports:
            print("Port %s" % p)
            if "CP210" in p[1]:
                prt = p[0]
            elif "Q_MPPC_CTL" in p[1]:
                prt = p[0]
        try:
            self.ser = serial.Serial(prt)  # open serial port
            self.ser.baudrate = 38400  # set baudrate
            self.ser.parity = serial.PARITY_EVEN  # set parity
            self.ser.stopbits = serial.STOPBITS_ONE
            self.ser.bytesize = serial.EIGHTBITS
            self.ser.timeout = 2.0
            print(self.ser.name)  # check which port was really used
        except NameError:
            print("No CP210x Controller was found. Check USB connection")

    def _write(self, command):
        data = command.encode()
        # print("command %s encoded %s" % (command, data))
        return self.ser.write(command.encode())

    def _read(self, length):
        rx = self.ser.read(length)
        if length == len(rx):
            procd = [c for c in rx]
            # print("rec: %s" % procd)
        else:
            print("short read %d vs %d" % (len(rx), length))
        return rx

    def _convert(self, command):
        # Converts command to hex form needed for serial data transfer
        com = command.encode(encoding="utf-8", errors="strict")
        command_str = binascii.hexlify(com).decode("utf-8")
        sum_command = sum(bytearray(com))
        return (command_str, sum_command)

    def _checksum(self, sum_command, sum_voltage):
        # CHECKSUM CALCULATION
        CS = hex(int(self.STX, 16) + sum_command + int(self.ETX, 16) + sum_voltage)
        CS = CS.lstrip("0x")
        CS = CS.upper()
        CS = CS[-2:]
        CS_str, CS_sum = self._convert(CS)
        return (CS_str, CS_sum)

    def _checkerror(self, rx):
        # Error Commands
        if rx == b"0001":
            print("UART communication error: Parity error, overrun error, framing error")
        if rx == b"0002":
            print(
                "Timeout error: This indicates that the self.CR has not been received within 1000ms of receiving the self.STX. The received packet is discarded."
            )
        if rx == b"0003":
            print(
                "Syntax error: The beginning of the received command is other than self.STX, which indicates the length of the command or 256byte."
            )
        if rx == b"0004":
            print("Checksum error: This indicates that the checksum does not match")
        if rx == b"0005":
            print("Command error: This indicates that it is an undefined command")
        if rx == b"0006":
            print("Parameter error: This indicates that the codes other than ASCII code(0~F) is in the parameter")
        if rx == b"0007":
            print(
                "Parameter size error: This indicates that the data length of the parameter is outside the specified length"
            )

    def _checkstatus(self, rx):
        # Status information
        status = int(rx[4:8])
        print("Status: %04x" % status)
        if status & 1:
            print("High Voltage Output      :   ON")
        else:
            print("High Voltage Output      :   OFF")

        if status & 2:
            print("Over-current protection  :   Working protection")
        else:
            print("Over-current protection  :   Not working")

        if status & 4:
            print("Current Value            :   Outside Specifications")
        else:
            print("Current Value            :   Within Specifications")

        if status & 8:
            print("MPPC temperature sensor  :   Connected")
        else:
            print("MPPC temperature sensor  :   Disconnected")

        if status & 0x10:
            print("MPPC temperature sensor  :   Outside Specifications")
        else:
            print("MPPC temperature sensor  :   Within Specifications")

        if status & 0x40:
            print("Temperature Correction   :   Effectiveness")
        else:
            print("Temperature Correction   :   Invalid")

        if status & 0x400:
            print("Automatic restoration    :   Restoration")
        else:
            print("Automatic restoration    :   Not working")

        if status & 0x800:
            print("Voltage suppression      :   Suppression")
        else:
            print("Voltage suppression   :   Not working")

        if status & 0x1000:
            print("Output voltage control   :   Control")
        else:
            print("Output voltage control   :   Not control")

        if status & 0x4000:
            print("Voltage stability        :   Stable")
        else:
            print("Voltage stability        :   Unstable")

    ##### COMMANDS OF POWER SUPPLY ####

    def help(self):
        print("printMonitorInfo() - Prints information on the power supply status, voltage (V) and current (mA) values")
        print("getPowerInfo() - Returns the power supply voltage (V) and current (mA) values as tuple")
        print("setHVOff() - Set power supply High Voltage OFF")
        print("setHVOn() - Set power supply High Voltage ON")
        print("reset()  - Reset the power supply")
        print("setVoltage(voltage_dec) - Sets the high voltage output to the voltage specified (V)")
        print("getVoltage() - Returns power supply voltage in Volts")
        print("getCurrent() - Returns power supply current in mA")
        print(
            "printStatus() - Prints status information on the power supply (similar to getMonitorInfo()) but without voltage and current values"
        )
        print("close() - Close serial port")
        print("help() - This help")

    def printMonitorInfo(self):
        "Prints information on the power supply status, voltage and current values"
        self.ser.flushInput()
        self.ser.flushOutput()
        command_str, sum_command = self._convert("HPO")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self.STX + command_str + self.ETX + CS_str + self.CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(28)
        if rx[1:4] == b"hpo":
            volt_out = int(rx[12:16], 16) * self.V_conversion
            mA_out = int(rx[16:20], 16) * self.I_conversion

            self._checkstatus(rx)
            print("High Voltage Output      :   {} V".format(volt_out))
            print("Output current           :   {} mA".format(mA_out))
        elif rx[1:4] == b"hxx":
            return self._checkerror(rx[4:8])
        else:
            print("An error has occured")

    def getPowerInfo(self):
        """
        Returns the power supply voltage and current values.
        Returns
        -------
        tuple
            (voltage, current)
            Voltage in Volts and current in mA.
        """
        self.ser.flushInput()
        self.ser.flushOutput()
        command_str, sum_command = self._convert("HPO")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self.STX + command_str + self.ETX + CS_str + self.CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(28)
        if rx[1:4] == b"hpo":
            volt_out = int(rx[12:16], 16) * self.V_conversion
            mA_out = int(rx[16:20], 16) * self.I_conversion
            return (volt_out, mA_out)
        elif rx[1:4] == b"hxx":
            return self._checkerror(rx[4:8])
        else:
            print("An error has occured")

    def setHVOff(self):
        "Set power supply High Voltage OFF"
        self.ser.flushInput()
        self.ser.flushOutput()
        command_str, sum_command = self._convert("HOF")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self.STX + command_str + self.ETX + CS_str + self.CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(8)
        if rx[1:4] == b"hof":
            pass
        elif rx[1:4] == b"hxx":
            return self._checkerror(rx[4:8])
        else:
            print("An error has occured")

    def setHVOn(self):
        "Set power supply High Voltage ON"
        self.ser.flushInput()
        self.ser.flushOutput()
        command_str, sum_command = self._convert("HON")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self.STX + command_str + self.ETX + CS_str + self.CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(8)
        if rx[1:4] == b"hon":
            pass
        elif rx[1:4] == b"hxx":
            return self._checkerror(rx[4:8])
        else:
            print("An error has occured")

    def reset(self):
        "Reset the power supply"
        self.ser.flushInput()
        self.ser.flushOutput()
        command_str, sum_command = self._convert("HRE")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self.STX + command_str + self.ETX + CS_str + self.CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(8)
        if rx[1:4] == b"hre":
            pass
        elif rx[1:4] == b"hxx":
            return self._checkerror(rx[4:8])
        else:
            print("An error has occured")

    def setVoltage(self, voltage_dec):
        """
        Sets the high voltage output to the voltage specified.
        Arguments
        ---------
        voltage_dec : float
            applied voltage
            NOTE -  The applied voltage can be set upto 90 V by c11204 power supply.
                    Change the upper voltage limit (self.V_lim_upper) as required by the MPPC in use
        """
        self.ser.flushInput()
        self.ser.flushOutput()
        if voltage_dec > self.V_lim_upper:
            print("Voltage is too high")
        elif voltage_dec < 40:
            print("Voltage is too low")
        else:
            voltage_conv = float(voltage_dec) / self.V_conversion
            voltage = int(round(voltage_conv))
            voltage_hex = hex(voltage)  # convert voltage from decimal to hexadecimal number
            voltage_hex = voltage_hex.lstrip("0x")
            voltage_str, sum_voltage = self._convert(voltage_hex)
            command_str, sum_command = self._convert("HBV")
            CS_str, CS_sum = self._checksum(sum_command, sum_voltage)

            # FINAL COMMAND
            command_tosend = self.STX + command_str + voltage_str + self.ETX + CS_str + self.CR
            command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
            # print("send command:" + command_x)
            tx = self._write(command_x)
            rx = self._read(8)
            if rx[1:4] == b"hbv":
                pass
            elif rx[1:4] == b"hxx":
                return self._checkerror(rx[4:8])
            else:
                print("An error has occured")

    def getVoltage(self):
        """
        Returns power supply voltage
        Returns
        -------
        float
            Voltage in Volts
        """
        self.ser.flushInput()
        self.ser.flushOutput()
        command_str, sum_command = self._convert("HGV")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self.STX + command_str + self.ETX + CS_str + self.CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(8)
        if rx[1:4] == b"hgv":
            volt_out = int(rx[4:8], 16) * self.V_conversion
            return volt_out
        elif rx[1:4] == b"hxx":
            return self._checkerror(rx[4:8])
        else:
            print("An error has occured")

    def getCurrent(self):
        """
        Returns power supply current
        Returns
        -------
        float
            Current in mA
        """
        self.ser.flushInput()
        self.ser.flushOutput()
        command_str, sum_command = self._convert("HGC")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self.STX + command_str + self.ETX + CS_str + self.CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(8)
        if rx[1:4] == b"hgc":
            I_out = int(rx[4:8], 16) * self.I_conversion
            return I_out
        elif rx[1:4] == b"hxx":
            return self._checkerror(rx[4:8])
        else:
            print("An error has occured")

    def printStatus(self):
        "Prints status information on the power supply (similar to getMonitorInfo()) but without voltage and current values"
        self.ser.flushInput()
        self.ser.flushOutput()
        command_str, sum_command = self._convert("HGS")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self.STX + command_str + self.ETX + CS_str + self.CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(8)
        if rx[1:4] == b"hgs":
            self._checkstatus(rx)
        elif rx[1:4] == b"hxx":
            return self._checkerror(rx[4:8])
        else:
            print("An error has occured")

    def close(self):
        "Close self.serial port"
        self.ser.close()
