import neopixel
from machine import Pin, ADC, I2C
import utime

from lib.pico_i2c_lcd import I2cLcd

# IO Constants (local LDRs on this Pico)
LDR_1 = ADC(28)
LDR_2 = ADC(27)
LDR_3 = ADC(26)

# Neopixel (optional feedback)
NP_LEN = 8
NP = neopixel.NeoPixel(Pin(22), NP_LEN)

# LCD (same I2C bus used for LCD only)
LCD_I2C_BUS = I2C(1, sda=Pin(2), scl=Pin(3), freq=400_000)
LCD_I2C_ADDR = LCD_I2C_BUS.scan()[0]
LCD = I2cLcd(LCD_I2C_BUS, LCD_I2C_ADDR, 2, 16)

# Start button
START = Pin(10, Pin.IN, Pin.PULL_UP)

# Game configuration
GAME_DURATION = 60.0
LOOP_S = 0.1

# States
STATE_WAITING = 0
STATE_PLAYING = 1
STATE_RESETTING = 2

LABUBU_COUNT = 3
LABUBU_MASK = (1 << LABUBU_COUNT) - 1

# Threshold: fraction of full-scale below which we consider the sensor "down"
LDR_DOWN_THRESHOLD = 0.3
_FULL16 = 65535

def read_single_ldr(adc: ADC) -> bool:
    """Return True if the labubu is up (1), False if down (0)."""
    val = adc.read_u16()
    is_down = val < int(LDR_DOWN_THRESHOLD * _FULL16)
    return not is_down

def read_ldr_byte() -> int:
    """Read three local LDRs and return a bitmask (bit0 = LDR_1, bit1 = LDR_2, bit2 = LDR_3)."""
    b = 0
    if read_single_ldr(LDR_1):
        b |= (1 << 0)
    if read_single_ldr(LDR_2):
        b |= (1 << 1)
    if read_single_ldr(LDR_3):
        b |= (1 << 2)
    return b

def lcd_show(time_left, score):
    LCD.clear()
    LCD.move_to(0, 0)
    LCD.putstr("Time: {:>3}s".format(int(time_left)))
    LCD.move_to(0, 1)
    LCD.putstr("Score: {:>3}".format(score))

def all_labubus_down(ldr_byte):
    return (ldr_byte & LABUBU_MASK) == 0

def main_loop():
    game_state = STATE_WAITING
    last_start_state = 1  # pull-up
    game_start_time = 0.0
    score = 0
    prev_ldr_byte = read_ldr_byte()

    while True:
        t0 = utime.ticks_ms() / 1e3

        start_pressed = START.value() == 0
        ldr_byte = read_ldr_byte()

        # Edge detect start button (high -> low)
        start_edge = False
        if last_start_state == 1 and start_pressed:
            start_edge = True
        last_start_state = 0 if start_pressed else 1

        if game_state == STATE_WAITING:
            if start_edge:
                # No homing step when using manual reset; start immediately
                score = 0
                game_start_time = utime.ticks_ms() / 1e3
                prev_ldr_byte = ldr_byte
                game_state = STATE_PLAYING
                lcd_show(GAME_DURATION, score)

        elif game_state == STATE_PLAYING:
            elapsed = (utime.ticks_ms() / 1e3) - game_start_time
            time_left = max(0.0, GAME_DURATION - elapsed)

            lcd_show(time_left, score)

            # Count 1 -> 0 transitions only while not resetting
            for i in range(LABUBU_COUNT):
                prev_bit = (prev_ldr_byte >> i) & 1
                cur_bit = (ldr_byte >> i) & 1
                if prev_bit == 1 and cur_bit == 0:
                    score += 1
            prev_ldr_byte = ldr_byte

            if all_labubus_down(ldr_byte):
                game_state = STATE_RESETTING
                LCD.clear()
                LCD.move_to(0, 0)
                LCD.putstr("Resetting...")
                LCD.move_to(0, 1)
                LCD.putstr("Use hands to reset")

            if elapsed >= GAME_DURATION:
                # End game
                game_state = STATE_WAITING
                lcd_show(0, score)
                utime.sleep(2.0)

        elif game_state == STATE_RESETTING:
            # Wait for any labubu to be put back up (bit becomes 1)
            if (ldr_byte & LABUBU_MASK) != 0:
                elapsed = (utime.ticks_ms() / 1e3) - game_start_time
                if elapsed < GAME_DURATION:
                    prev_ldr_byte = ldr_byte
                    game_state = STATE_PLAYING
                else:
                    game_state = STATE_WAITING

        # loop pacing
        t1 = utime.ticks_ms() / 1e3
        loop_elapsed = t1 - t0
        if loop_elapsed < LOOP_S:
            utime.sleep(LOOP_S - loop_elapsed)

if __name__ == "__main__":
    main_loop()