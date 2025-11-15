from machine import Pin, I2CTarget

class EasyI2CTarget:
    """Easy I2C target with a callback when memory is written by controller."""
    
    def __init__(self, addr, i2c_id, scl_pin, sda_pin, mem_size_bytes):
        self.__mem = bytearray(mem_size_bytes)
        self.i2c = I2CTarget(id=i2c_id, addr=addr, scl=scl_pin, sda=sda_pin, mem=self.__mem)

    def write_mem(self, memaddr, value):
        self.__mem[memaddr] = value

    def write_multi(self, memaddr, value, length):
        self.__mem[memaddr:memaddr+length] = value

    def read_mem(self, memaddr):
        return self.__mem[memaddr]
    
    def read_multi(self, memaddr, length):
        return int.from_bytes(self.__mem[memaddr:memaddr+length], byteorder='big', signed=False)

    def write_bit(self, memaddr, bitpos, value):
        if value:
            self.__mem[memaddr] |= 1 << bitpos
        else:
            self.__mem[memaddr] &= ~(1 << bitpos)

    def read_bit(self, memaddr, bitpos):
        return (self.__mem[memaddr] >> bitpos) & 1