import utime
from machine import ADC

from constants import *
from lib.debounce import Debouncer
from lib.easy_i2c_target import EasyI2CTarget
from lib.motor_controller import MotorController
from lib.task import Task
from lib.task_scheduler import TaskScheduler

ADDRESS = 0x20

# Row 1 specific LDR thresholds
LABUBU_1_DOWN_THRESHOLD = 0.013
LABUBU_2_DOWN_THRESHOLD = 0.015
LABUBU_3_DOWN_THRESHOLD = 0.015

# Reset val is duty cycle to push up the labubu
MOTOR_1_RESET_VAL = 2.0
MOTOR_2_RESET_VAL = 2.0

RESET_FORWARD_TIME = 2.0
RESET_BACKWARD_TIME = 2.0

# Create I2C target with shared constants
I2C_TARGET = EasyI2CTarget(
    addr=ADDRESS,
    i2c_id=I2C_BUS_ID,
    scl_pin=BUS_SCL,
    sda_pin=BUS_SDA,
    mem_size_bytes=I2C_MEM_SIZE,
)

# Create hardware instances using shared pin definitions
LDR_1 = ADC(LDR_1_PIN)
LDR_2 = ADC(LDR_2_PIN)
LDR_3 = ADC(LDR_3_PIN)

MOTOR_1 = MotorController(MOTOR_1_PWM_PIN, MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN)
MOTOR_2 = MotorController(MOTOR_2_PWM_PIN, MOTOR_2_IN1_PIN, MOTOR_2_IN2_PIN)

# Task Scheduler setup
scheduler = TaskScheduler()
motor_resource = scheduler.create_resource("motors")

# State variables
labubus_reset = False
motors_resetting = False


class ResetLabubus(Task):
    def __init__(self, motor_1, motor_2, motor_resource):
        super().__init__(priority=1, name="ResetLabubus")
        self.add_resources(motor_resource)
        self.start_time = 0
        self.phase = 0
        self.motor_1 = motor_1
        self.motor_2 = motor_2

    def start(self, timestamp: float):
        # Set global resetting state
        global motors_resetting, labubus_reset
        labubus_reset = False
        motors_resetting = True

        self.start_time = timestamp
        self.phase = 0
        self.motor_1.set_duty_cycle(MOTOR_1_RESET_VAL)
        self.motor_2.set_duty_cycle(MOTOR_2_RESET_VAL)

    def execute(self, timestamp: float):
        if timestamp > (self.start_time + RESET_FORWARD_TIME) and self.phase == 0:
            self.phase = 1
            self.motor_1.set_duty_cycle(-MOTOR_1_RESET_VAL)
            self.motor_2.set_duty_cycle(-MOTOR_2_RESET_VAL)

    def isFinished(self, timestamp: float) -> bool:
        # Reset for total configured time
        return (timestamp - self.start_time) >= (
            RESET_FORWARD_TIME + RESET_BACKWARD_TIME
        )

    def end(self, canceled: bool, timestamp: float):
        # Update homing state and resetting state
        global labubus_reset, motors_resetting
        if not canceled:
            labubus_reset = True
        else:
            labubus_reset = False
        motors_resetting = False

        self.motor_1.stop()
        self.motor_2.stop()


last_cmd_state = 0

debouncers = [Debouncer(False), Debouncer(False), Debouncer(False)]

while True:
    time_start = utime.ticks_ms() / 1e3
    scheduler.run(time_start)

    labubus_down = [
        debouncer.calculate(val)
        for (val, debouncer) in zip(
            [
                read_labubu_down(LDR_1, LABUBU_1_DOWN_THRESHOLD),
                read_labubu_down(LDR_2, LABUBU_2_DOWN_THRESHOLD),
                read_labubu_down(LDR_3, LABUBU_3_DOWN_THRESHOLD),
            ],
            debouncers,
        )
    ]
    # Send output state (1 for up, 0 for down)
    I2C_TARGET.write_bit(0, LABUBU_1, not labubus_down[0])
    I2C_TARGET.write_bit(0, LABUBU_2, not labubus_down[1])
    I2C_TARGET.write_bit(0, LABUBU_3, not labubus_down[2])

    # Check if the labubus are all reset
    labubus_reset = labubus_reset and not any(labubus_down)

    I2C_TARGET.write_bit(1, RESET, labubus_reset)
    I2C_TARGET.write_bit(1, RESETTING, motors_resetting)

    print("Labubu 1 down: ", I2C_TARGET.read_bit(0, LABUBU_1))
    print("Labubu 2 down: ", I2C_TARGET.read_bit(0, LABUBU_2))
    print("Labubu 3 down: ", I2C_TARGET.read_bit(0, LABUBU_3))

    # Read commands
    cmd_state = I2C_TARGET.read_mem(2)
    if cmd_state != last_cmd_state:
        # Reset command
        if (cmd_state & (1 << RESET_CMD)) and not (last_cmd_state & (1 << RESET_CMD)):
            if not motors_resetting:
                reset_task = ResetLabubus(MOTOR_1, MOTOR_2, motor_resource)
                scheduler.schedule(reset_task)

        last_cmd_state = cmd_state

    time_end = utime.ticks_ms() / 1e3
    elapsed = time_end - time_start
    if elapsed < LOOP_INTERVAL:
        utime.sleep(LOOP_INTERVAL - elapsed)
    else:
        print("Warning: Loop overrun: took ", str(elapsed), "seconds")


