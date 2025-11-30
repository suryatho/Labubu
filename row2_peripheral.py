import utime
from machine import ADC

from constants import *
from lib.easy_i2c_target import EasyI2CTarget
from lib.motor_controller import MotorController
from lib.task import Task
from lib.task_scheduler import TaskScheduler

# Row 2 specific configuration
ADDRESS = 0x20

# Row 2 specific LDR thresholds (different from row 1)
LABUBU_1_DOWN_THRESHOLD = 0.4
LABUBU_2_DOWN_THRESHOLD = 0.35
LABUBU_3_DOWN_THRESHOLD = 0.25

# Row 2 specific motor control values (different from row 1)
MOTOR_1_HOME_VAL = 0.6
MOTOR_2_HOME_VAL = -0.4
MOTOR_1_RESET_VAL = -0.4
MOTOR_2_RESET_VAL = 0.6

# Row 2 specific timing values (different from row 1)
HOME_TIME = 2.5
RESET_FORWARD_TIME = 1.8
RESET_BACKWARD_TIME = 2.2

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
motors_homed = False
motors_homing = False
motors_resetting = False


class HomeMotors(Task):
    def __init__(self, motor_1, motor_2, motor_resource):
        super().__init__(priority=1, name="HomeMotors")
        self.add_resources(motor_resource)
        self.start_time = 0
        self.motor_1 = motor_1
        self.motor_2 = motor_2

    def start(self, timestamp: float):
        # Set global homing state
        global motors_homed, motors_homing
        motors_homed = False
        motors_homing = True

        self.start_time = timestamp
        self.motor_1.set_duty_cycle(MOTOR_1_HOME_VAL)
        self.motor_2.set_duty_cycle(MOTOR_2_HOME_VAL)

    def isFinished(self, timestamp: float) -> bool:
        # Home for configured time
        return (timestamp - self.start_time) >= HOME_TIME

    def end(self, canceled: bool, timestamp: float):
        # Set global homing state
        global motors_homed, motors_homing
        if not canceled:
            motors_homed = True
        motors_homing = False

        self.motor_1.stop()
        self.motor_2.stop()


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
        global motors_resetting
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
        global motors_homed, motors_resetting
        if not canceled:
            motors_homed = True
        else:
            motors_homed = False
        motors_resetting = False

        self.motor_1.stop()
        self.motor_2.stop()


last_cmd_state = 0
while True:
    time_start = utime.ticks_ms() / 1e3
    scheduler.run(time_start)

    # Send output state (1 for down, 0 for up)
    I2C_TARGET.write_bit(0, LABUBU_1, read_labubu_down(LDR_1, LABUBU_1_DOWN_THRESHOLD))
    I2C_TARGET.write_bit(0, LABUBU_2, read_labubu_down(LDR_2, LABUBU_2_DOWN_THRESHOLD))
    I2C_TARGET.write_bit(0, LABUBU_3, read_labubu_down(LDR_3, LABUBU_3_DOWN_THRESHOLD))
    I2C_TARGET.write_bit(1, HOMED, motors_homed)
    I2C_TARGET.write_bit(1, HOMING, motors_homing)
    I2C_TARGET.write_bit(1, RESETTING, motors_resetting)

    # Read commands
    cmd_state = I2C_TARGET.read_mem(2)
    if cmd_state != last_cmd_state:
        # Home command
        if (cmd_state & (1 << HOME_CMD)) and not (last_cmd_state & (1 << HOME_CMD)):
            if not motors_homing:
                home_task = HomeMotors(MOTOR_1, MOTOR_2, motor_resource)
                scheduler.schedule(home_task)

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
