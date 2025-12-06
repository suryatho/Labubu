from machine import Pin

# I2C Protocol Constants - Bit positions in memory
LABUBU_1 = 0
LABUBU_2 = 1
LABUBU_3 = 2

RESET = 0
RESETTING = 1

RESET_CMD = 0

# I2C Bus Pin Configuration (same for all peripherals)
BUS_SCL = Pin(3)
BUS_SDA = Pin(2)
I2C_BUS_ID = 1
I2C_MEM_SIZE = 3

# Hardware Pin Assignments (same for all peripherals)
LDR_1_PIN = 26
LDR_2_PIN = 27
LDR_3_PIN = 28

MOTOR_1_PWM_PIN = 16
MOTOR_1_IN1_PIN = 17
MOTOR_1_IN2_PIN = 18

MOTOR_2_PWM_PIN = 15
MOTOR_2_IN1_PIN = 14
MOTOR_2_IN2_PIN = 13

# Loop Timing
LOOP_INTERVAL = 0.01


def read_labubu_down(ldr, threshold):
    """Check if a labubu is down based on LDR reading"""
    return ldr.read_u16() < (threshold * 65535)
