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
from serial.serialutil import SerialException
import serial.tools.list_ports
import binascii


class CLAWSps:
    """Class defining an interface to a Hamamatsu c11204-01/02 power supply as
    used with the CLAWS detectors.

    Attributes:
        max_voltage: Maximum allowed voltage to be set. Chose wisely depending on your hardware.
        min_voltage: Minimum allowed voltage to be set.
    """
    def __init__(self, serial_port: (str | int) = 0, max_voltage: float = 60, min_voltage: float = 40):
        """Constructor to set up connection via serial UART interface.

        Args:
            serial_port: Descriptor of the serial port (as e.g. "/dev/ttyUSB0")
            or integer indexing all found serial ports that contain 'CP210' or
            'Q_MPPC_CTL' in their name.
            max_voltage: Maximum allowed voltage to be set. Chose wisely depending on your hardware.
            min_voltage: Minimum allowed voltage to be set.

        Raises:
            SerialException: Raised when no serial ports could be discovered on
            the system. This is the case when no serial device is connected
            (any device whatsoever, not only the power supply).
        """
        # Internally used fixed values
        self._STX = "02"  # start of text
        self._ETX = "03"  # end of text
        self._CR = "0D"  # carriage return (i.e. command end)

        # Conversion factors for readings from the ps
        self._voltage_conversion = 1.812 * 10 ** (-3)  # voltage conversion factor
        self._current_conversion = 4.980 * 10 ** (-3)  # current conversion factor (mA)

        # Internally used variables
        self._ser = 0  # Reference to serial port
        user_def_sp = serial_port  # Use a more meaningful name for this context

        # User defined variables
        self.max_voltage = max_voltage  # in V
        self.min_voltage = min_voltage  # in V

        # Open serial port
        ports = list(serial.tools.list_ports.comports())  # Get available ports
        if len(ports) == 0:
            raise SerialException("No serial ports found. Check device connections.")

        if type(user_def_sp) is int:
            # Choose nth port found of name matching what is expected for the ps
            ps_ports = [p for p in ports if ("CP210" in p[1]) or ("Q_MPPC_CTL" in p[1])]
            prt_name = ps_ports[user_def_sp][0]
        elif type(user_def_sp) is str:
            prt_name = user_def_sp  # Choose based on name like `/dev/ttyUSB0`

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

    def _write(self, command: str) -> (int | None):
        """Write to serial port registered with this instance.

        Args:
            command: Command to write in string representation.

        Returns:
            Return value of serial.Serial() write() method.
        """
        # Write to serial device
        return self._ser.write(command.encode())

    def _read(self, length: int) -> bytes:
        """Read from serial port registered with this instance.

        Args:
            length: Number of bytes to read.

        Returns:
            Bytes as returned by serial.Serial() read() method.
        """
        rx = self._ser.read(length)
        if length != len(rx):  # Not sure if this is really needed, but keep for now
            print(f"Short read: {len(rx)} vs {length}")
        return rx

    def _convert_command(self, command):
        # Converts command to hex form needed for serial data transfer
        command_bytes = command.encode()
        command_hex = command_bytes.hex()  # Get hex representation of command
        command_checksum = sum(bytearray(command_bytes))  # Compute checksum of command
        return command_hex, command_checksum

    def _send_serial_command(self, command: str, value=0, response_length: int = 8):
        self._ser.flushInput()
        self._ser.flushOutput()
        command_hex, command_checksum = self._convert_command(command)
        value_hex = hex(value).lstrip("0x")
        voltage_hex, voltage_checksum = self._convert_command(value_hex)  # Value str is empty iv value=0

        # Calculate checksum. Presumably following the procedure prescribed in
        # the manual.
        total_checksum = hex(int(self._STX, 16) + command_checksum + int(self._ETX, 16) + voltage_checksum)
        total_checksum = total_checksum.lstrip("0x").upper()[-2:]
        total_checksum_str, _ = self._convert_command(total_checksum)

        # Assemble final command
        command_to_send = self._STX + command_hex + voltage_hex + self._ETX + total_checksum_str + self._CR
        command_x = "".join(chr(int(command_to_send[n: n + 2], 16)) for n in range(0, len(command_to_send), 2))
        self._write(command_x)
        rx = self._read(response_length)
        return rx

    def _send_serial_command_checkresp(
        self, command: str, value=0, command_response: (str | None) = None, response_length: int = 8
    ) -> bytes:
        # Additionally check the response and raise an error accordingly if
        # some error is reported.
        if command_response is None:
            # The usual response is the same as the command, except all
            # lowercase. Commands are usually all uppercase.
            command_response = command.lower()
        rx = self._send_serial_command(command, value, response_length)
        if rx[1:4] == command_response.encode():
            return rx
        elif rx[1:4] == b"hxx":
            self._checkerror(rx[4:8])  # Must raise an error!
        else:
            raise serial.SerialException("Unexpected respose.")

    def hv_disable(self) -> bytes:
        "Set power supply High Voltage OFF"
        return self._send_serial_command_checkresp("HOF")

    def hv_enable(self) -> bytes:
        "Set power supply High Voltage ON"
        return self._send_serial_command_checkresp("HON")

    def reset(self) -> bytes:
        "Reset the power supply"
        return self._send_serial_command_checkresp("HRE")

    def set_voltage(self, voltage_dec) -> bytes:
        """
        Sets the high voltage output to the voltage specified.
        Arguments
        ---------
        voltage_dec : float
            applied voltage
            NOTE -  The applied voltage can be set upto 90 V by c11204 power supply.
                    Change the upper voltage limit (self.V_lim_upper) as required by the MPPC in use
        """
        if voltage_dec > self.max_voltage:
            raise ValueError(
                f"Voltage was set to {voltage_dec}, which is out of the defined voltage range ({self.min_voltage}, {self.max_voltage}). Voltage is too high."
            )
        elif voltage_dec < self.min_voltage:
            raise ValueError(
                f"Voltage was set to {voltage_dec}, which is out of the defined voltage range ({self.min_voltage}, {self.max_voltage}). Voltage is too low."
            )
        else:
            voltage_conv = float(voltage_dec) / self._voltage_conversion
            value = int(round(voltage_conv))
            return self._send_serial_command_checkresp("HBV", value)

    def get_voltage(self) -> float:
        """
        Returns power supply voltage
        Returns
        -------
        float
            Voltage in Volts
        """
        rx = self._send_serial_command_checkresp("HGV")
        voltage = int(rx[4:8], 16) * self._voltage_conversion
        return voltage

    def get_current(self) -> float:
        """
        Returns power supply current
        Returns
        -------
        float
            Current in mA
        """
        rx = self._send_serial_command_checkresp("HGC")
        current = int(rx[4:8], 16) * self._current_conversion
        return current

    def get_power(self) -> float:
        # Gets voltage and current at the same time. This could technically be
        # more precise than calling HGV and HGC in succession, but the effect
        # is probably negligible here.
        rx = self._send_serial_command_checkresp("HPO", response_length=28)
        voltage = int(rx[12:16], 16) * self._voltage_conversion  # in V
        current_mA = int(rx[16:20], 16) * self._current_conversion  # in mA
        return voltage * current_mA  # in mW

    def get_temperature(self) -> float:
        # TODO: The forumla in the command reference I have is a little
        # unclear. Also, for CLAWS currently we use no temperature sensor.
        # Despite of this the status says it's connected, which is a little
        # concerning. Investigate this and maybe implement this function.
        # Also consider funcitons for setting temperature compensation modes
        # for completness.

        # rx = self._send_serial_command_checkresp("HGT")
        # temperature = self._temperature_conversion(int(rx[4:8], 16))
        # return temperature
        raise NotImplementedError

    def close(self):
        "Close self.serial port"
        self._ser.close()

    def get_status_raw(self) -> int:
        # Can format nicely with something like `"%04x" % status_raw`
        rx = self._send_serial_command_checkresp("HGS")
        return int(rx[4:8])  # Return the int encoding the status

    def get_status(self) -> dict:
        "Prints status information on the power supply (similar to getMonitorInfo()) but without voltage and current values"
        status_hex = self.get_status_raw()
        return self.parse_status(status_hex)

    def _checkerror(self, rx):
        # Raise exceptions indicating errors as described in manual
        if rx == b"0001":
            raise SerialException("UART communication error: Parity error, overrun error, framing error")
        elif rx == b"0002":
            raise SerialException(
                "Timeout error: This indicates that the self.CR has not been received within 1000ms of receiving the self.STX. The received packet is discarded."
            )
        elif rx == b"0003":
            raise SerialException(
                "Syntax error: The beginning of the received command is other than self.STX, which indicates the length of the command or 256byte."
            )
        elif rx == b"0004":
            raise SerialException("Checksum error: This indicates that the checksum does not match")
        elif rx == b"0005":
            raise SerialException("Command error: This indicates that it is an undefined command")
        elif rx == b"0006":
            raise SerialException(
                "Parameter error: This indicates that the codes other than ASCII code(0~F) is in the parameter"
            )
        elif rx == b"0007":
            raise SerialException(
                "Parameter size error: This indicates that the data length of the parameter is outside the specified length"
            )

    def parse_status(self, status: int) -> dict:
        # Use shorthand to make the following code less convoluted
        s = status

        # TODO: Look up status codes and check if the namings are appropriate
        status_dict = {
            "high_voltage_output": bool(s & 1),
            "overcurrent_protection": bool(s & 2),
            "current_in_spec": not bool(s & 4),
            "MPPC_temp_sensor_connected": bool(s & 8),
            "MPPC_temp_sensor_in_spec": not bool(s & 0x10),
            "temperature_correction": bool(s & 0x40),
            "automatic_restore_enabled": bool(s & 0x400),
            "voltage_suppression": bool(s & 0x800),
            "output_voltage_control": bool(s & 0x1000),
            "voltage_stable": bool(s & 0x4000),
        }
        return status_dict
