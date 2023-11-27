# pyCLAWSps

Python package for controlling a Hamamatsu c11204-01/02 power supply.

## Installation
This version of the package is not yet distributed to the package index.
The existing entry of `pyCLAWSps` is a much older version. This code is a
hopefully cleaner rewrite of the original `pyCLAWSps` code.

## Compatibility
The code was developed for a power supply board for the CLAWS scintillation
detectors and was only ever tested in this context. However, technically
essentially all functionality should be general, as long as the serial UART
interface of the c11204-01/02 can be accessed.

## Using the package
Simply instantiate a `C11204PS` object and call any of the implemented functions.
The functions themselves should be documented with docstrings in the code.
An example to set and check the voltage is the following:
```py
from pyCLAWSps import C11204PS
# Call with no arguments initializes with first serial port with name
# containing 'CP210' or 'Q_MPPC_CTL'.
ps = C11204PS()  
ps.hv_disable()  # Disable voltage supply
print(ps.get_voltage())  # Should read close to zero

ps.set_voltage(56.7)  # Set the voltage (choose appropriate value)

ps.hv_enable()
print(ps.get_voltage())  # Should read close to set voltage

print(ps.get_status())  # Print dict of status flags for the power supply
```
