"""
Typed wrapper for pyserial to handle typing issues with platform-specific implementations.
"""
import re
import time
from typing import Any, Optional, Union

import serial
from serial.tools import list_ports  # type: ignore


class RHSPSerial:
    """Typed wrapper around serial.Serial with proper type annotations."""
    
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 9600,
        bytesize: int = 8,
        parity: str = 'N',
        stopbits: float = 1,
        timeout: Optional[float] = None,
        xonxoff: bool = False,
        rtscts: bool = False,
        write_timeout: Optional[float] = None,
        dsrdtr: bool = False,
        inter_byte_timeout: Optional[float] = None,
        exclusive: Optional[bool] = None,
        **kwargs: Any
    ) -> None:
        """Initialize the serial port wrapper."""
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout,
            xonxoff=xonxoff,
            rtscts=rtscts,
            write_timeout=write_timeout,
            dsrdtr=dsrdtr,
            inter_byte_timeout=inter_byte_timeout,
            exclusive=exclusive,
            **kwargs
        )
    
    def open(self) -> None:
        """Open the serial port."""
        self._serial.open()  # type: ignore
    
    def close(self) -> None:
        """Close the serial port."""
        self._serial.close()  # type: ignore
    
    def isOpen(self) -> bool:
        """Check if the serial port is open."""
        return self._serial.isOpen()  # type: ignore
    
    def is_open(self) -> bool:
        """Check if the serial port is open (modern property name)."""
        return self._serial.is_open  # type: ignore
    
    def read(self, size: int = 1) -> bytes:
        """Read data from the serial port."""
        return self._serial.read(size)  # type: ignore
    
    def write(self, data: Union[bytes, bytearray]) -> int:
        """Write data to the serial port."""
        return self._serial.write(data)  # type: ignore
    
    def inWaiting(self) -> int:
        """Get the number of bytes waiting in the input buffer."""
        return self._serial.inWaiting()  # type: ignore
    
    def in_waiting(self) -> int:
        """Get the number of bytes waiting in the input buffer (modern property name)."""
        return self._serial.in_waiting  # type: ignore
    
    def flush(self) -> None:
        """Flush the write buffer."""
        self._serial.flush()  # type: ignore
    
    def flushInput(self) -> None:
        """Flush the input buffer."""
        self._serial.flushInput()  # type: ignore
    
    def flushOutput(self) -> None:
        """Flush the output buffer."""
        self._serial.flushOutput()  # type: ignore
    
    def reset_input_buffer(self) -> None:
        """Reset the input buffer."""
        self._serial.reset_input_buffer()  # type: ignore
    
    def reset_output_buffer(self) -> None:
        """Reset the output buffer."""
        self._serial.reset_output_buffer()  # type: ignore
    
    @property
    def port(self) -> Optional[str]:
        """Get the port name."""
        return self._serial.port  # type: ignore
    
    @port.setter
    def port(self, value: Optional[str]) -> None:
        """Set the port name."""
        self._serial.port = value  # type: ignore
    
    @property
    def baudrate(self) -> int:
        """Get the baudrate."""
        return self._serial.baudrate  # type: ignore
    
    @baudrate.setter
    def baudrate(self, value: int) -> None:
        """Set the baudrate."""
        self._serial.baudrate = value  # type: ignore
    
    @property
    def bytesize(self) -> int:
        """Get the bytesize."""
        return self._serial.bytesize  # type: ignore
    
    @bytesize.setter
    def bytesize(self, value: int) -> None:
        """Set the bytesize."""
        self._serial.bytesize = value  # type: ignore
    
    @property
    def parity(self) -> str:
        """Get the parity."""
        return self._serial.parity  # type: ignore
    
    @parity.setter
    def parity(self, value: str) -> None:
        """Set the parity."""
        self._serial.parity = value  # type: ignore
    
    @property
    def stopbits(self) -> float:
        """Get the stopbits."""
        return self._serial.stopbits  # type: ignore
    
    @stopbits.setter
    def stopbits(self, value: float) -> None:
        """Set the stopbits."""
        self._serial.stopbits = value  # type: ignore
    
    @property
    def timeout(self) -> Optional[float]:
        """Get the timeout."""
        return self._serial.timeout  # type: ignore
    
    @timeout.setter
    def timeout(self, value: Optional[float]) -> None:
        """Set the timeout."""
        self._serial.timeout = value  # type: ignore
    
    # Context manager support
    def __enter__(self) -> 'RHSPSerial':
        """Enter the context manager."""
        return self
    
    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """Exit the context manager."""
        self.close()


class comPort:

    def __init__(self, sn, name, device ):
        self.sn = sn
        self.name = name
        self.device = device


    def number_from_name(self):
        num = re.findall('\\d+', self.name)
        return num[-1]


    @classmethod
    def enumerate(cls) -> list["comPort"]:
        ports = []
        for usbDevice in list_ports.comports():

            if 'SER=' in usbDevice.hwid:

                _sections = usbDevice.hwid.split(' ')
                sections = dict([section.split('=') for section in _sections if '=' in section])

                if sections.get('SER','').startswith('D'):
                    serialNumber = sections['SER']
                    deviceName = usbDevice.device
                    time.sleep(0.2)
                    ports.append(cls(serialNumber, deviceName, usbDevice))

        return ports
