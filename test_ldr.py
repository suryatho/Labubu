import utime
from machine import ADC

from constants import *

LDR_1 = ADC(LDR_1_PIN)
LDR_2 = ADC(LDR_2_PIN)
LDR_3 = ADC(LDR_3_PIN)


def read_ldr(ldr):
    return ldr.read_u16() / ((1 << 16) - 1)


while True:
    print("LDR 1:", read_ldr(LDR_1))
    print("LDR 2:", read_ldr(LDR_2))
    print("LDR 3:", read_ldr(LDR_3))
    utime.sleep(0.5)
