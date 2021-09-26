import enum
from collections import defaultdict
import pyftdi.spi


class Operation:
    Write = 0
    Read = 1


class BankMode:
    Paired = 0
    Individual = 1


class OperationMode:
    Sequential = 0
    Byte = 1


class Register(enum.Enum):
    IODIRA = {
        BankMode.Individual: 0x00,
        BankMode.Paired: 0x00
    }
    IODIRB = {
        BankMode.Individual: 0x10,
        BankMode.Paired: 0x01
    }
    IPOLA = {
        BankMode.Individual: 0x01,
        BankMode.Paired: 0x02
    }
    IPOLB = {
        BankMode.Individual: 0x11,
        BankMode.Paired: 0x03
    }
    GPINTENA = {
        BankMode.Individual: 0x02,
        BankMode.Paired: 0x04
    }
    GPINTENB = {
        BankMode.Individual: 0x12,
        BankMode.Paired: 0x05
    }
    DEFVALA = {
        BankMode.Individual: 0x03,
        BankMode.Paired: 0x06
    }
    DEFVALB = {
        BankMode.Individual: 0x13,
        BankMode.Paired: 0x07
    }
    INTCONA = {
        BankMode.Individual: 0x04,
        BankMode.Paired: 0x08
    }
    INTCONB = {
        BankMode.Individual: 0x14,
        BankMode.Paired: 0x09
    }
    IOCON = {
        BankMode.Individual: 0x05,
        BankMode.Paired: 0x0A
    }
    IOCON2 = {
        BankMode.Individual: 0x15,
        BankMode.Paired: 0x0B
    }
    GPPUA = {
        BankMode.Individual: 0x06,
        BankMode.Paired: 0x0C
    }
    GPPUB = {
        BankMode.Individual: 0x16,
        BankMode.Paired: 0x0D
    }
    INTFA = {
        BankMode.Individual: 0x07,
        BankMode.Paired: 0x0E
    }
    INTFB = {
        BankMode.Individual: 0x17,
        BankMode.Paired: 0x0F
    }
    INTCAPA = {
        BankMode.Individual: 0x08,
        BankMode.Paired: 0x10,
    }
    INTCAPB = {
        BankMode.Individual: 0x18,
        BankMode.Paired: 0x11
    }
    GPIOA = {
        BankMode.Individual: 0x09,
        BankMode.Paired: 0x12
    }
    GPIOB = {
        BankMode.Individual: 0x19,
        BankMode.Paired: 0x13
    }
    OLATA = {
        BankMode.Individual: 0x0A,
        BankMode.Paired: 0x14
    }
    OLATB = {
        BankMode.Individual: 0x1A,
        BankMode.Paired: 0x15
    }

    def nextAddress(currentRegister, operationMode, bankMode):
        # Sequential mode enables automatic address pointer incrementing
        if operationMode == OperationMode.Sequential:
            nextAddress = (currentRegister.value[bankMode] + 1) % 0x16
            for i in Register:
                if i.value[bankMode] == nextAddress:
                    return i
        # Byte mode with IOCON.BANK = 0 causes the address pointer to toggle between associated A/B register pairs
        elif (operationMode == OperationMode.Byte and bankMode == BankMode.Paired):
            # IOCON register pair does not follow xA/xB naming schema
            if currentRegister.name == 'IOCON':
                return Register.IOCON2
            elif currentRegister.name == 'IOCON2':
                return Register.IOCON

            # Flip A to B or B to A
            nextRegisterName = currentRegister.name[:-1]
            nextRegisterName += 'A' if currentRegister.name[-1] == 'B' else 'B'
            for i in Register:
                if i.name == nextRegisterName:
                    return i
            raise Exception(f"Register {nextRegisterName} not exists")
        return currentRegister


class Mask(enum.Enum):
    BANK = (1 << 7)
    MIRROR = (1 << 6)
    SEQOP = (1 << 5)
    DISSLW = (1 << 4)
    HAEN = (1 << 3)
    ODR = (1 << 2)
    INTPOL = (1 << 1)


class PinMode:
    Output = 0
    Input = 1


class MCP23S17:
    def __init__(self, deviceId, ftdiDevice='ftdi://ftdi:2232h/1', freq=8E6):
        self._cntrl = pyftdi.spi.SpiController()
        self._cntrl.configure(ftdiDevice)
        self.spi = self._cntrl.get_port(cs=0, freq=freq, mode=0)
        self.deviceId = deviceId

        # Default mode when a chip is reset
        self.bankMode = BankMode.Paired
        self.operationMode = OperationMode.Sequential

        self.cache = defaultdict(int)
        self.cache[Register.IODIRA] = 0xFF
        self.cache[Register.IODIRB] = 0xFF

    def init(self):
        # Sequential operation disabled, address pointer does not increment
        # Enables the MCP23S17 address pins
        self.writeRegister(Register.IOCON, Mask.SEQOP.value | Mask.HAEN.value)
        self.operationMode = OperationMode.Byte
        self.readAllRegistersToCache()

    def readAllRegistersToCache(self):
        for register in Register:
            self.readRegister(register)

    def pinMode(self, pin, mode):
        assert(pin >= 0 and pin <= 15)
        register = Register.IODIRA

        if pin >= 8:
            register = Register.IODIRB
            pin -= 8

        pinMask = 1 << pin
        self.writeRegisterBit(register, pinMask, mode)

    def writePin(self, pin, value):
        assert(pin >= 0 and pin <= 15)
        register = Register.GPIOA

        if pin >= 8:
            register = Register.GPIOB
            pin -= 8
        pinMask = 1 << pin
        self.writeRegisterBit(register, pinMask, value)

    def readPin(self, pin):
        assert(pin >= 0 and pin <= 15)
        register = Register.GPIOA

        if pin >= 8:
            register = Register.GPIOB
            pin -= 8

        pinMask = 1 << pin
        return 1 if self.readRegister(register) & pinMask else 0

    def writePortA(self, value):
        self.writeRegister(Register.GPIOA, value)

    def writePortB(self, value):
        self.writeRegister(Register.GPIOB, value)

    def writePort(self, value):
        assert(self.operationMode ==
               OperationMode.Byte and self.bankMode == BankMode.Paired)
        portA = value & 0xff
        portB = (value >> 8) & 0xff
        self.writeRegister(Register.GPIOA, [portA, portB])

    def readPortA(self):
        return self.readRegister(Register.GPIOA)

    def readPortB(self):
        return self.readRegister(Register.GPIOB)

    def readPort(self):
        return self.readPortA + (self.readPortB() << 8)

    def enableAddressPins(self):
        self.setRegisterBit(Register.IOCON, Mask.HAEN)

    def disableAddressPins(self):
        self.clearRegisterBit(Register.IOCON, Mask.HAEN)

    def writeRegister(self, register, value):
        self.cache[register] = value
        if type(register) == Register:
            address = register.value[self.bankMode]
        else:
            address = register

        if type(value) == int:
            value = [value]

        data = [self._deviceOpcode(Operation.Write), address] + value
        self.spi.exchange(data, 0)

        # Update cache
        for i in value:
            self.cache[register] = i
            register = Register.nextAddress(
                register, self.operationMode, self.bankMode)

    def readRegister(self, register, count=1):
        if type(register) == Register:
            address = register.value[self.bankMode]
        else:
            address = register

        data = [self._deviceOpcode(Operation.Read), address]
        res = self.spi.exchange(data, count)

        # Update chache
        cacheReg = register
        for i in res:
            self.cache[register] = i
            cacheReg = Register.nextAddress(
                register, self.operationMode, self.bankMode)

        if count == 1:
            return res[0]
        else:
            return res

    def writeRegisterBit(self, register, mask, value):
        if isinstance(mask, enum.Enum):
            mask = mask.value

        if value == 0:
            self.cache[register] &= ~mask
        else:
            self.cache[register] |= mask
        self.writeRegister(register, self.cache[register])

    def setRegisterBit(self, register, mask):
        self.writeRegisterBit(register, mask, 1)

    def clearRegisterBit(self, register, mask):
        self.writeRegisterBit(register, mask, 0)

    def _deviceOpcode(self, operation):
        FIXED_ADDRESS = 0x40
        return FIXED_ADDRESS + (self.deviceId << 1) + operation


if __name__ == '__main__':
    import time

    mcp = MCP23S17(0x00)
    mcp.init()  # Enable HAEN and disable SeqOp

    # Set pins 0..7 (port A) to outputs
    # same as mcp.writeRegister(Register.IODIRA, 0x00)
    for i in range(0, 8):
        mcp.pinMode(i, PinMode.Output)

    # Set pins 8..15 (port B) to inputs
    # same as mcp.writeRegister(Register.IODIRB, 0xff)
    for i in range(8, 16):
        mcp.pinMode(i, PinMode.Input)

   # With IOCON.SEQOP = 1 (seq write disabled) and IOCON.BANK = 0 (registers are paired)
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
