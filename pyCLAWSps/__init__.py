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

from typing import Tuple
import serial
import serial.tools.list_ports
import binascii
import numpy as np


class CLAWSps:
    # NOTE - The applied voltage can be set upto 90 V by c11204 power supply.
    # Change the upper voltage limit (self.V_lim_upper) as required by the MPPC in use
    def __init__(self, serial:Tuple[str, int], max_voltage:float = 60):
        # Internally used fixed values
        self._STX = "02"  # start of text
        self._ETX = "03"  # end of text
        self._CR = "0D"  # delimiter
        # Conversion factors for readings from the ps
        self._voltage_conversion = 1.812 * 10 ** (-3)  # voltage conversion factor
        self._current_conversion = 4.980 * 10 ** (-3)  # current conversion factor (mA)

        # Internally used variables
        self._ser = 0  # Reference to serial port
        user_def_serial = serial  # Use a more meaningful name for this context

        # User defined variables
        self.max_voltage = max_voltage  # Upper high voltage limit in Volts
        self._min_voltage = 40

        # Open serial port
        ports = list(serial.tools.list_ports.comports())  # Get available ports

        if type(user_def_serial) is int:
            # Choose nth port found of name matching what is expected for the ps
            ps_ports = [p for p in ports if ("CP210" in p[1]) or ("Q_MPPC_CTL" in p[1])]
            prt_name = ps_ports[user_def_serial][0]
        elif type(user_def_serial) is str:
            prt_name = user_def_serial  # Choose based on name like `/dev/ttyUSB0`

        try:
            self._ser = serial.Serial(prt_name)  # open serial port
            self._ser.baudrate = 38400  # set baudrate
            self._ser.parity = serial.PARITY_EVEN  # set parity
            self._ser.stopbits = serial.STOPBITS_ONE
            self._ser.bytesize = serial.EIGHTBITS
            self._ser.timeout = 2.0
            print(f"Setup finished for serial prt '{self._ser.name}'")  # Confirm the used serial port
        except Exception as e:
            print("Serial setup failed with unhandled exception.")
            raise e

    def _write(self, command):
        # Write to serial device
        return self._ser.write(command.encode())

    def _read(self, length):
        # Read from serial device
        rx = self._ser.read(length)
        if length != len(rx):  # Not sure if this is really needed, but keep for now
            print(f"Short read: {len(rx)} vs {length}")
        return rx

    def _convert(self, command):
        # Converts command to hex form needed for serial data transfer
        com = command.encode(encoding="utf-8", errors="strict")
        command_str = binascii.hexlify(com).decode("utf-8")
        sum_command = sum(bytearray(com))
        return (command_str, sum_command)

    def _checksum(self, command, value):
        # Calculate checksum
        cs = hex(int(self._STX, 16) + command + int(self._ETX, 16) + value)
        cs = cs.lstrip("0x")
        cs = cs.upper()
        cs = cs[-2:]
        cs_str, cs_sum = self._convert(cs)
        return (cs_str, cs_sum)

    def _send_serial_command(self, command:str, value=0):
        self._ser.flushInput()
        self._ser.flushOutput()
        command_str, sum_command = self._convert(command)
        value_hex = hex(value)  # convert voltage from decimal to hexadecimal number
        value_hex = value_hex.lstrip("0x")
        value_str, sum_voltage = self._convert(value_hex)  # Value str is empty iv value=0
        command_str, sum_command = self._convert("HBV")
        cs_str, cs_sum = self._checksum(sum_command, sum_voltage)

        # Assemble final command
        command_tosend = self._STX + command_str + value_str + self._ETX + cs_str + self._CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(8)
        return rx
       
    def _send_serial_command_checkresp(self, command:str, value=0, command_response:Tuple[str, None] = None):
        # Additionally check the response and raise an error accordingly if
        # some error is reported.
        if command_response is None:
            # The usual response is the same as the command, except all
            # lowercase. Commands are usually all uppercase.
            command_response = command.lower()
        rx = self._send_serial_command(command)
        if rx[1:4] == command_response.encode():
            return rx
        elif rx[1:4] == b"hxx":
            self._checkerror(rx[4:8])  # Must raise an error!
        else:
            raise serial.SerialException("Unexpected respose.")

    def setHVOff(self):
        "Set power supply High Voltage OFF"
        return self._send_serial_command_checkresp("HOF")
       
    def setHVOn(self):
        "Set power supply High Voltage ON"
        return self._send_serial_command_checkresp("HON")

    def reset(self):
        "Reset the power supply"
        return self._send_serial_command_checkresp("HRE")
       
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
        # TODO: raise errors accordingly
        if voltage_dec > self.max_voltage:
            print("Voltage is too high")
        elif voltage_dec < self._min_voltage:
            print("Voltage is too low")
        else:
            voltage_conv = float(voltage_dec) / self._voltage_conversion
            value = int(round(voltage_conv))
            return self._send_serial_command_checkresp("HBV", value)

    def getVoltage(self):
        """
        Returns power supply voltage
        Returns
        -------
        float
            Voltage in Volts
        """
        return self._send_serial_command_checkresp("HGV")

    def getCurrent(self):
        """
        Returns power supply current
        Returns
        -------
        float
            Current in mA
        """
        return self._send_serial_command_checkresp("HGC")

    def close(self):
        "Close self.serial port"
        self._ser.close()

    def printStatus(self):
        "Prints status information on the power supply (similar to getMonitorInfo()) but without voltage and current values"
        self._ser.flushInput()
        self._ser.flushOutput()
        command_str, sum_command = self._convert("HGS")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self._STX + command_str + self._ETX + CS_str + self._CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(8)
        if rx[1:4] == b"hgs":
            self.parse_status(rx)
        elif rx[1:4] == b"hxx":
            return self._checkerror(rx[4:8])
        else:
            print("An error has occured")

    def _checkerror(self, rx):
        # TODO: Make this nicer by defining proper exceptions
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

    def parse_status(self, rx) -> dict:
        # TODO: define a get_status function to just get status dict, no
        # arguments required. Do this when cleaning up the funcions calling
        # this funciton.

        # Status information
        s = int(rx[4:8])

        print("Status: %04x" % s)

        # TODO: Look up status codes and check if the namings are appropriate
        d = {
                "high_voltage_output" : bool(s & 1),
                "overcurrent_protection":  bool(s & 2),
                "current_in_spec": not bool(s & 4),
                "MPPC_temp_sensor_connected": bool(s & 8),
                "MPPC_temp_sensor_in_spec": not bool(s & 0x10),
                "temperature_correction": bool(s & 0x40),
                "automatic_restore_enabled": bool(s & 0x400),
                "voltage_suppression": bool(s & 0x800),
                "output_voltage_control": bool(s & 0x1000),
                "voltage_stable": bool(s & 0x4000)
                }

        print(d)

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
        self._ser.flushInput()
        self._ser.flushOutput()
        command_str, sum_command = self._convert("HPO")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self._STX + command_str + self._ETX + CS_str + self._CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(28)
        if rx[1:4] == b"hpo":
            volt_out = int(rx[12:16], 16) * self._voltage_conversion
            mA_out = int(rx[16:20], 16) * self._current_conversion

            self.parse_status(rx)
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
        self._ser.flushInput()
        self._ser.flushOutput()
        command_str, sum_command = self._convert("HPO")
        CS_str, CS_sum = self._checksum(sum_command, 0)

        # FINAL COMMAND
        command_tosend = self._STX + command_str + self._ETX + CS_str + self._CR
        command_x = "".join(chr(int(command_tosend[n : n + 2], 16)) for n in range(0, len(command_tosend), 2))
        tx = self._write(command_x)
        rx = self._read(28)
        if rx[1:4] == b"hpo":
            volt_out = int(rx[12:16], 16) * self._voltage_conversion
            mA_out = int(rx[16:20], 16) * self._current_conversion
            return (volt_out, mA_out)
        elif rx[1:4] == b"hxx":
            return self._checkerror(rx[4:8])
        else:
            print("An error has occured")
