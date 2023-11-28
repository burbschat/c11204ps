# c11204ps
[![PyPI - Version](https://img.shields.io/pypi/v/c11204ps.svg)](https://pypi.org/project/c11204ps)
[![PyPI - Python Version](https://img.shields.io/pypi/pyversions/c11204ps.svg)](https://pypi.org/project/c11204ps)

Python package for controlling a Hamamatsu c11204-01/02 power supply.
This is essentially a re-write of [`pyCLAWSps`](https://github.com/malindasds/pyCLAWSps).
While the `pyCLAWSps` was fine for basic control of a c11204 power supply, this
package aims to provide a more general, streamlined python interface.

For the reason of this being based on `pyCLAWSps`, the version count for this
package starts at `2.0`.

## Installation
This package should be in the package index and installable with pip.
```console
pip install c11204ps
```

Alternatively build with `hatch` and install from `.whl`.
Run from the root of this repository
```console
hatch build
pip install ./dist/c11204ps-0.2.0-py3-none-any.whl
```
Adjust filename for current verison if neccessary.

## Compatibility
The code was developed for a power supply board for the CLAWS scintillation
detectors and was only ever tested in this context. However, technically pretty
much all functionality should be general, making this package usable as long as
the c11204-01/02 serial UART interface can be accessed.

## Using the package
Simply instantiate a `C11204PS` object and call any of the implemented functions.
The functions themselves should be documented with docstrings in the code.
An example to set and check the voltage is the following:
```py
from c11204ps import C11204PS
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

## License
c11204ps is distributed under the terms of the [GPL-3.0-only](https://spdx.org/licenses/GPL-3.0-only.html) license.
