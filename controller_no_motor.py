import neopixel
from machine import I2C, Pin
import utime

from lib.pico_i2c_lcd import I2cLcd

# I2C Bus Constants
I2C_SCL = Pin(5)
I2C_SDA = Pin(4)
I2C_BUS = I2C(0, sda=I2C_SDA, scl=I2C_SCL, freq=400_000)

print(I2C_BUS.scan())
ROW_1_ADDR = 0x10
if ROW_1_ADDR not in I2C_BUS.scan():
    raise Exception("Row 1 peripheral not found on I2C bus!")

# IO Constants
# NP_LEN = 8
# NP = neopixel.NeoPixel(Pin(22), NP_LEN)

LCD_I2C_BUS = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
addresses = LCD_I2C_BUS.scan()
LCD_I2C_ADDR = None
# while LCD_I2C_ADDR is None:
for addr in addresses:
    if addr != ROW_1_ADDR:
        LCD_I2C_ADDR = addr
            #print("Found LCD at address:", hex(LCD_I2C_ADDR))
LCD = I2cLcd(LCD_I2C_BUS, LCD_I2C_ADDR, 2, 16)

START = Pin(13, Pin.IN, Pin.PULL_UP)

def read_row_ldr_state(row_addr):
    data = I2C_BUS.readfrom_mem(row_addr, 0, 1)
    return int.from_bytes(data, 'big')

# Game configuration
GAME_DURATION = 60.0
LOOP_S = 0.1

# States
STATE_WAITING = 0
STATE_PLAYING = 1
STATE_RESETTING = 2

LABUBU_COUNT = 3
LABUBU_MASK = (1 << LABUBU_COUNT) - 1

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
    last_start_state = 1  # PULL_UP
    game_start_time = 0
    score = 0
    prev_ldr_byte = read_row_ldr_state(ROW_1_ADDR)

    while True:
        t0 = utime.ticks_ms() / 1e3

        start_pressed = START.value() == 0
        ldr_byte = read_row_ldr_state(ROW_1_ADDR)

        # start button edge detect
        start_edge = False
        if last_start_state == 1 and start_pressed:
            start_edge = True
        last_start_state = 0 if start_pressed else 1

        if game_state == STATE_WAITING:
            if start_edge:
                # No motors: skip homing and start immediately
                score = 0
                game_start_time = utime.ticks_ms() / 1e3
                prev_ldr_byte = ldr_byte
                game_state = STATE_PLAYING

        elif game_state == STATE_PLAYING:
            elapsed = (utime.ticks_ms() / 1e3) - game_start_time
            time_left = max(0.0, GAME_DURATION - elapsed)

            lcd_show(time_left, score)

            # Count 1->0 transitions as points (only while not resetting)
            for i in range(LABUBU_COUNT):
                prev_bit = (prev_ldr_byte & (1 << i)) != 0
                cur_bit = (ldr_byte & (1 << i)) != 0
                if prev_bit and not cur_bit:
                    score += 1
            prev_ldr_byte = ldr_byte

            if all_labubus_down(ldr_byte):
                # Enter manual resetting state (hands will reset)
                game_state = STATE_RESETTING
                # show resetting message briefly
                LCD.clear()
                LCD.move_to(0, 0)
                LCD.putstr("Resetting...")
                LCD.move_to(0, 1)
                LCD.putstr("Hold to reset")

            if elapsed >= GAME_DURATION:
                # end game
                game_state = STATE_WAITING
                lcd_show(0, score)
                utime.sleep(2.0)

        elif game_state == STATE_RESETTING:
            # Wait for a human reset: detect any labubu going back up (bit becomes 1)
            if (ldr_byte & LABUBU_MASK) != 0:
                # resume playing if time remains, else go to waiting
                elapsed = (utime.ticks_ms() / 1e3) - game_start_time
                if elapsed < GAME_DURATION:
                    prev_ldr_byte = ldr_byte
                    game_state = STATE_PLAYING
                else:
                    game_state = STATE_WAITING

        # pacing
        t1 = utime.ticks_ms() / 1e3
        elapsed = t1 - t0
        if elapsed < LOOP_S:
            utime.sleep(LOOP_S - elapsed)

if __name__ == "__main__":
    main_loop()