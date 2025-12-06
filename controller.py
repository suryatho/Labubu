import sys

import neopixel
import utime
from machine import I2C, Pin

from lib.pico_i2c_lcd import I2cLcd

# I2C Bus Constants
I2C_SCL = Pin(3)
I2C_SDA = Pin(2)
I2C_BUS = I2C(1, sda=I2C_SDA, scl=I2C_SCL, freq=400_000)

ROW_1_ADDR = 0x10
ROW_2_ADDR = 0x20
while ROW_1_ADDR not in I2C_BUS.scan():
    print("Row 1 peripheral not found on I2C bus!")
    utime.sleep(0.1)
while ROW_2_ADDR not in I2C_BUS.scan():
    print("Row 2 peripheral not found on I2C bus!")
    utime.sleep(0.1)
ROW_ADDRESSES = [ROW_1_ADDR, ROW_2_ADDR]

LCD_I2C_BUS = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
LCD_I2C_ADDR = LCD_I2C_BUS.scan()[0]
print(LCD_I2C_ADDR)
LCD = I2cLcd(LCD_I2C_BUS, LCD_I2C_ADDR, 2, 16)


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
STATE_PREPARING = 1
STATE_PLAYING = 2
STATE_ENDING = 3

RESET_CMD_BIT = 0
LABUBU_COUNT = 3
LABUBU_MASK = (1 << LABUBU_COUNT) - 1


def write_bit(addr, byte_addr, bit_pos, value):
    # Read current byte, update the bit, write back
    try:
        cur = I2C_BUS.readfrom_mem(addr, byte_addr, 1)[0]
    except Exception:
        cur = 0
    if value:
        new = cur | (1 << bit_pos)
    else:
        new = cur & ~(1 << bit_pos)
    # write back as single byte
    I2C_BUS.writeto_mem(addr, 2, bytes([new]))


def write_cmd_bit(addr, bit_pos, value):
    write_bit(addr, 2, bit_pos, value)


def lcd_show(time_left, score):
    # First line: Time left
    LCD.move_to(0, 0)
    LCD.putstr("Time: {:>3}s".format(int(time_left)))
    # Second line: Score
    LCD.move_to(0, 1)
    LCD.putstr("Score: {:>3}".format(score))


def lcd_show_waiting():
    LCD.move_to(0, 0)
    LCD.putstr("Press Enter")
    LCD.move_to(0, 1)
    LCD.putstr("to start game!")


def all_labubus_down(ldr_byte):
    return (ldr_byte & LABUBU_MASK) == 0


def wait_for_game_start():
    """Wait for user to press Enter to start a new game"""
    LCD.clear()
    lcd_show_waiting()
    print("Press Enter to start a new game...")
    input()  # Blocking wait for Enter key
    print("Starting new game...")


def main_loop():
    game_state = STATE_WAITING
    game_start_time = 0
    game_end_time = 0
    score = 0
    # Keep previous LDR per-labubu bits to detect 1 -> 0 transitions
    prev_ldr_bytes = [read_row_ldr_state(ROW_1_ADDR), read_row_ldr_state(ROW_2_ADDR)]
    row_resetting = [False, False]

    # Track when we need to update the display
    last_time_shown = -1
    last_score_shown = -1

    while True:
        t0 = utime.ticks_ms() / 1e3

        if game_state == STATE_WAITING:
            wait_for_game_start()
            # Wait for enter to start
            game_state = STATE_PREPARING

        elif game_state == STATE_PREPARING:
            # Read peripheral inputs
            motor_bytes = [
                read_row_motor_state(ROW_1_ADDR),
                # read_row_motor_state(ROW_2_ADDR),
            ]

            for addr in ROW_ADDRESSES:
                write_cmd_bit(addr, RESET_CMD_BIT, 1)

            all_reset = True
            for i, addr in enumerate(ROW_ADDRESSES):
                reset_flag = (motor_bytes[i] & 1) != 0
                if not reset_flag:
                    all_reset = False
                    break

            if all_reset:
                # Clear the home command for both rows and start the game
                for addr in ROW_ADDRESSES:
                    write_cmd_bit(addr, RESET_CMD_BIT, 0)
                score = 0
                game_start_time = utime.ticks_ms() / 1e3
                ldr_bytes = [
                    read_row_ldr_state(ROW_1_ADDR),
                    read_row_ldr_state(ROW_2_ADDR),
                ]
                prev_ldr_bytes = ldr_bytes[:]
                row_resetting = [
                    False,
                    # False
                ]
                game_state = STATE_PLAYING

                # Clear LCD once for game display
                LCD.clear()
                last_time_shown = -1
                last_score_shown = -1
                print("Game started! Knock down the labubus!")

        elif game_state == STATE_PLAYING:
            # Read peripheral inputs
            ldr_bytes = [read_row_ldr_state(ROW_1_ADDR), read_row_ldr_state(ROW_2_ADDR)]
            motor_bytes = [
                read_row_motor_state(ROW_1_ADDR),
                read_row_motor_state(ROW_2_ADDR),
            ]

            elapsed = (utime.ticks_ms() / 1e3) - game_start_time
            time_left = max(0.0, GAME_DURATION - elapsed)

            # Update LCD only when time or score changes (reduce flickering)
            time_to_show = int(time_left)
            if time_to_show != last_time_shown or score != last_score_shown:
                lcd_show(time_left, score)
                last_time_shown = time_to_show
                last_score_shown = score

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
                            print(f"Hit! Score: {score}")
                    prev_ldr_bytes[row_idx] = ldr_byte

                # Check if all labubus are down in this row and start reset if needed
                if all_labubus_down(ldr_byte) and not row_resetting[row_idx]:
                    write_cmd_bit(addr, RESET_CMD_BIT, 1)
                    row_resetting[row_idx] = True
                    print(f"Resetting row {row_idx + 1}")

                # Check if this row finished resetting
                elif row_resetting[row_idx] and not resetting_flag:
                    # Reset completed for this row
                    write_cmd_bit(addr, RESET_CMD_BIT, 0)
                    row_resetting[row_idx] = False
                    prev_ldr_bytes[row_idx] = read_row_ldr_state(addr)
                    print(f"Row {row_idx + 1} reset complete")

            # End game when time is up
            if elapsed >= GAME_DURATION:
                game_state = STATE_ENDING
                game_end_time = utime.ticks_ms() / 1e3
                print(f"Game over! Final score: {score}")
                # Clear LCD once for end game display
                LCD.clear()

        elif game_state == STATE_ENDING:
            # Display final score for 5 seconds, then go to waiting
            elapsed_end = (utime.ticks_ms() / 1e3) - game_end_time
            countdown = 5.0 - elapsed_end

            # Show final score with countdown
            LCD.move_to(0, 0)
            LCD.putstr("Final: {:>3}".format(score))
            LCD.move_to(0, 1)
            LCD.putstr("Next: {:>3}s".format(int(countdown)))

            if elapsed_end >= 5.0:
                for addr in ROW_ADDRESSES:
                    write_cmd_bit(addr, RESET_CMD_BIT, 0)
                row_resetting = [False, False]
                game_state = STATE_WAITING

        # Small loop pacing (but not when waiting for input)
        if game_state != STATE_WAITING:
            t1 = utime.ticks_ms() / 1e3
            elapsed = t1 - t0
            if elapsed < LOOP_S:
                utime.sleep(LOOP_S - elapsed)


if __name__ == "__main__":
    print("Labubu Game Controller")
    print("Press Enter to start a new game")
    main_loop()
