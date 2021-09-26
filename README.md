# pyftdi-mcp23s17

The Python module for a MCP23S17 GPIO expander, which is driven by [pyftdi](https://eblot.github.io/pyftdi/).

## Example

```python
import time
from mcp23s17 import MCP23S17, PinMode

mcp = MCP23S17(0x00)  # 0x00 is the device ID
mcp.init()  # Enable HAEN and disable SeqOp

# Set pins 0..7 (port A) to outputs
# same as mcp.writeRegister(Register.IODIRA, 0x00)
for i in range(0, 8):
    mcp.pinMode(i, PinMode.Output)

# Set pins 8..15 (port B) to inputs
# same as mcp.writeRegister(Register.IODIRB, 0xff)
for i in range(8, 16):
    mcp.pinMode(i, PinMode.Input)

# With IOCON.SEQOP = 1 (automatic address increment disabled) and IOCON.BANK = 0 (registers are paired)
# the above could be done:
# mcp.writeRegister(Register.IODIRA, [0x00, 0xff])
# in this mode the sequential write/read toggles between the register pair

mcp.writePortA(0x00)
while True:
    mcp.writePin(0, 0)
    time.sleep(1)
    mcp.writePin(0, 1)
    time.sleep(1)
    print("GPB0: ", mcp.readPin(8))
    print("GPB: ", mcp.readPortB())
```

