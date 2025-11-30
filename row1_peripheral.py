import utime
from machine import ADC, Pin

from lib.easy_i2c_target import EasyI2CTarget
from lib.motor_controller import MotorController
from lib.task import Task
from lib.task_scheduler import TaskScheduler

# I2C Bus Constants
ADDRESS = 0x10
BUS_SCL = Pin(5)
BUS_SDA = Pin(4)
I2C_TARGET = EasyI2CTarget(
    addr=ADDRESS, i2c_id=0, scl_pin=BUS_SCL, sda_pin=BUS_SDA, mem_size_bytes=3
)

LABUBU_1 = 0
LABUBU_2 = 1
LABUBU_3 = 2

HOMED = 0
HOMING = 1
RESETTING = 2

HOME_CMD = 0
RESET_CMD = 1

# IO Constants
LDR_1 = ADC(26)
LDR_2 = ADC(27)
LDR_3 = ADC(28)

MOTOR_1_PWM = 16
MOTOR_1_IN1 = 17
MOTOR_1_IN2 = 18
MOTOR_1 = MotorController(MOTOR_1_PWM, MOTOR_1_IN1, MOTOR_1_IN2)

MOTOR_2_PWM = 15
MOTOR_2_IN1 = 14
MOTOR_2_IN2 = 13
MOTOR_2 = MotorController(MOTOR_2_PWM, MOTOR_2_IN1, MOTOR_2_IN2)

# Task Scheduler setup
scheduler = TaskScheduler()
loop_s = 0.01

motor_resource = scheduler.create_resource("motors")
motors_homed = False
motors_homing = False
motors_resetting = False


class HomeMotors(Task):
    motor_1_home_val = 0.5
    motor_2_home_val = -0.5
    home_time = 2.0

    def __init__(self, motor_1, motor_2, motor_resource):
        super().__init__(priority=1, name="HomeMotors")
        self.add_resources(motor_resource)
        self.start_time = 0
        self.motor_1 = motor_1
        self.motor_2 = motor_2

    def start(self, timestamp: float):
        # Set global homing state
        global motors_homed
        motors_homed = False
        global motors_homing
        motors_homing = True

        self.start_time = timestamp
        self.motor_1.set_duty_cycle(HomeMotors.motor_1_home_val)
        self.motor_2.set_duty_cycle(HomeMotors.motor_2_home_val)

    def isFinished(self, timestamp: float) -> bool:
        # Home for 2 seconds
        return (timestamp - self.start_time) >= HomeMotors.home_time

    def end(self, canceled: bool, timestamp: float):
        # Set global homing state
        if not canceled:
            global motors_homed
            motors_homed = True
        global motors_homing
        motors_homing = False

        self.motor_1.stop()
        self.motor_2.stop()


class ResetLabubus(Task):
    reset_forward_time = 2.0
    reset_backward_time = 2.0
    motor_1_reset_val = -0.5
    motor_2_reset_val = 0.5

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
        self.motor_1.set_duty_cycle(ResetLabubus.motor_1_reset_val)
        self.motor_2.set_duty_cycle(ResetLabubus.motor_2_reset_val)

    def execute(self, timestamp: float):
        if (
            timestamp > (self.start_time + ResetLabubus.reset_forward_time)
            and self.phase == 0
        ):
            self.phase = 1
            self.motor_1.set_duty_cycle(-ResetLabubus.motor_1_reset_val)
            self.motor_2.set_duty_cycle(-ResetLabubus.motor_2_reset_val)

    def isFinished(self, timestamp: float) -> bool:
        # Reset for 4 seconds total
        return (timestamp - self.start_time) >= (
            ResetLabubus.reset_forward_time + ResetLabubus.reset_backward_time
        )

    def end(self, canceled: bool, timestamp: float):
        # Update homing state and resetting state
        global motors_homed
        if not canceled:
            motors_homed = True
        else:
            motors_homed = False
        global motors_resetting
        motors_resetting = False

        self.motor_1.stop()
        self.motor_2.stop()


def read_labubu_down(ldr, value):
    return ldr.read_u16() < (value * 65535)


LABUBU_1_DOWN_THRESHOLD = 0.3
LABUBU_2_DOWN_THRESHOLD = 0.3
LABUBU_3_DOWN_THRESHOLD = 0.3

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
    if elapsed < loop_s:
        utime.sleep(loop_s - elapsed)
    else:
        print("Warning: Loop overrun: took ", str(elapsed), "seconds")
