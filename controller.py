import neopixel
import utime
from machine import I2C, Pin

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
ROW_ADDRESSES = [ROW_1_ADDR, ROW_2_ADDR]

LCD_I2C_BUS = I2C(1, sda=Pin(2), scl=Pin(3), freq=400_000)
LCD_I2C_ADDR = LCD_I2C_BUS.scan()[0]
LCD = I2cLcd(LCD_I2C_BUS, LCD_I2C_ADDR, 2, 16)

START = Pin(10, Pin.IN, Pin.PULL_UP)


def read_row_ldr_state(row_addr):
    data = I2C_BUS.readfrom_mem(row_addr, 0, 1)
    return int.from_bytes(data, "big")


def read_row_motor_state(row_addr):
    data = I2C_BUS.readfrom_mem(row_addr, 1, 1)
    return int.from_bytes(data, "big")


# Game configuration
GAME_DURATION = 60.0  # seconds for a single game session
LOOP_S = 0.1

# Row / game state constants
STATE_WAITING = 0
STATE_HOMING = 1
STATE_PLAYING = 2
STATE_ENDING = 3

HOME_CMD_BIT = 0
RESET_CMD_BIT = 1
LABUBU_COUNT = 3
LABUBU_MASK = (1 << LABUBU_COUNT) - 1


def write_cmd_bit(addr, bit_pos, value):
    # Read current command byte (mem 2), update the bit, write back
    try:
        cur = I2C_BUS.readfrom_mem(addr, 2, 1)[0]
    except Exception:
        cur = 0
    if value:
        new = cur | (1 << bit_pos)
    else:
        new = cur & ~(1 << bit_pos)
    # write back as single byte
    I2C_BUS.writeto_mem(addr, 2, bytes([new]))


def lcd_show(time_left, score):
    LCD.clear()
    # First line: Time left
    LCD.move_to(0, 0)
    LCD.putstr("Time: {:>3}s".format(int(time_left)))
    # Second line: Score
    LCD.move_to(0, 1)
    LCD.putstr("Score: {:>3}".format(score))


def all_labubus_down(ldr_byte):
    return (ldr_byte & LABUBU_MASK) == 0


def main_loop():
    game_state = STATE_WAITING
    last_start_state = 1  # PULL_UP btw
    game_start_time = 0
    game_end_time = 0
    score = 0
    # Keep previous LDR per-labubu bits to detect 1 -> 0 transitions
    prev_ldr_bytes = [read_row_ldr_state(ROW_1_ADDR), read_row_ldr_state(ROW_2_ADDR)]
    row_resetting = [False, False]

    while True:
        t0 = utime.ticks_ms() / 1e3

        # Read inputs
        start_pressed = START.value() == 0
        ldr_bytes = [read_row_ldr_state(ROW_1_ADDR), read_row_ldr_state(ROW_2_ADDR)]
        motor_bytes = [
            read_row_motor_state(ROW_1_ADDR),
            read_row_motor_state(ROW_2_ADDR),
        ]

        # Debounce/edge detect for start button: detect high->low transition
        start_edge = False
        if last_start_state == 1 and start_pressed:
            start_edge = True
        last_start_state = 0 if start_pressed else 1

        if game_state == STATE_WAITING:
            # Wait for start button press
            if start_edge:
                # Begin homing sequence: set HOME command bit until homed
                game_state = STATE_HOMING
                for addr in ROW_ADDRESSES:
                    write_cmd_bit(addr, HOME_CMD_BIT, 1)

        elif game_state == STATE_HOMING:
            all_homed = True
            for i, addr in enumerate(ROW_ADDRESSES):
                homed_flag = (motor_bytes[i] & 1) != 0
                if not homed_flag:
                    all_homed = False
                    break

            if all_homed:
                # Clear the home command for both rows and start the game
                for addr in ROW_ADDRESSES:
                    write_cmd_bit(addr, HOME_CMD_BIT, 0)
                score = 0
                game_start_time = utime.ticks_ms() / 1e3
                prev_ldr_bytes = ldr_bytes[:]
                row_resetting = [False, False]
                game_state = STATE_PLAYING

        elif game_state == STATE_PLAYING:
            elapsed = (utime.ticks_ms() / 1e3) - game_start_time
            time_left = max(0.0, GAME_DURATION - elapsed)

            # Update LCD with remaining time and score
            lcd_show(time_left, score)

            # Check each row for scoring and reset conditions
            for row_idx, addr in enumerate(ROW_ADDRESSES):
                ldr_byte = ldr_bytes[row_idx]
                motor_byte = motor_bytes[row_idx]

                # Add point when a labubu flips from 1 to 0
                # Only count when this row is not resetting
                resetting_flag = (motor_byte & 0b100) != 0
                if not resetting_flag:
                    for i in range(LABUBU_COUNT):
                        prev_bit = (prev_ldr_bytes[row_idx] & (1 << i)) != 0
                        cur_bit = (ldr_byte & (1 << i)) != 0
                        if prev_bit == 1 and cur_bit == 0:
                            score += 1
                    prev_ldr_bytes[row_idx] = ldr_byte

                # Check if all labubus are down in this row and start reset if needed
                if all_labubus_down(ldr_byte) and not row_resetting[row_idx]:
                    write_cmd_bit(addr, RESET_CMD_BIT, 1)
                    row_resetting[row_idx] = True

                # Check if this row finished resetting
                elif row_resetting[row_idx] and not resetting_flag:
                    # Reset completed for this row
                    write_cmd_bit(addr, RESET_CMD_BIT, 0)
                    row_resetting[row_idx] = False
                    prev_ldr_bytes[row_idx] = read_row_ldr_state(addr)

            # End game when time is up
            if elapsed >= GAME_DURATION:
                game_state = STATE_ENDING
                game_end_time = utime.ticks_ms() / 1e3

        elif game_state == STATE_ENDING:
            # Display final score for 2 seconds, then go to waiting
            elapsed_end = (utime.ticks_ms() / 1e3) - game_end_time
            lcd_show(5.0 - elapsed_end, score)

            if elapsed_end >= 5.0:
                # Now clear all commands and reset state
                for addr in ROW_ADDRESSES:
                    write_cmd_bit(addr, HOME_CMD_BIT, 0)
                    write_cmd_bit(addr, RESET_CMD_BIT, 0)
                row_resetting = [False, False]
                game_state = STATE_WAITING

        # small loop pacing
        t1 = utime.ticks_ms() / 1e3
        elapsed = t1 - t0
        if elapsed < LOOP_S:
            utime.sleep(LOOP_S - elapsed)


if __name__ == "__main__":
    main_loop()
