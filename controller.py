from machine import Pin, I2C
import neopixel
from lib.pico_i2c_lcd import I2cLcd

# I2C Bus Constants
I2C_SCL = Pin(5)
I2C_SDA = Pin(4)
I2C_BUS = I2C(0, sda=I2C_SDA, scl=I2C_SCL, freq=400_000)

ROW_1_ADDR = 0x10
ROW_2_ADDR = 0x20
if ROW_1_ADDR not in I2C_BUS.scan():
    raise Exception("Row 1 peripheral not found on I2C bus!")
if ROW_2_ADDR not in I2C_BUS.scan():
    raise Exception("Row 2 peripheral not found on I2C bus!")

# IO Constants
NP_LEN = 8
NP = neopixel.NeoPixel(Pin(22), NP_LEN)

LCD_I2C_BUS = I2C(1, sda=Pin(2), scl=Pin(3), freq=400_000)
LCD_I2C_ADDR = LCD_I2C_BUS.scan()[0]
LCD = I2cLcd(LCD_I2C_BUS, LCD_I2C_ADDR, 2, 16)

def read_row_ldr_state(row_addr):
    data = I2C_BUS.readfrom_mem(row_addr, 0, 1)
    return data

def read_row_motor_state(row_addr):
    data = I2C_BUS.readfrom_mem(row_addr, 1, 1)
    return data

while True:
    pass