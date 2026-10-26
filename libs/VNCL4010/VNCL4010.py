

VCNL4010_I2C_ADDRESS = 0x13


# Register addresses

COMMAND_REG = 0x80

PROX_RATE_REG = 0x82

PROX_DATA_REG = 0x87

PROX_CURRENT_REG = 0x89

INTERRUPT_CTRL_REG = 0x8B


class VCNL4010:

    def __init__(self, i2c, addr=VCNL4010_I2C_ADDRESS):

        self.i2c = i2c

        self.addr = addr

        self.begin()

    def write8(self, reg, value):

        self.i2c.writeto_mem(self.addr, reg, bytes([value]))

    def read8(self, reg):

        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def read16(self, reg):

        data = self.i2c.readfrom_mem(self.addr, reg, 2)

        return (data[0] << 8) | data[1]

    def begin(self):

        self.write8(PROX_CURRENT_REG, 0x20)

        self.write8(PROX_RATE_REG, 0x02)

        time.sleep(0.1)

    def read_proximity(self):

        return self.read16(PROX_DATA_REG)
