from machine import PWM, Pin


class MotorController:
    def __init__(self, pwm, in1, in2):
        self.pwm = PWM(Pin(pwm))
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.pwm.freq(1000)

    def set_duty_cycle(self, duty_cycle):
        self.in1.value(1 if duty_cycle > 0 else 0)
        self.in2.value(0 if duty_cycle > 0 else 1)
        duty_16 = int(abs(duty_cycle) * 65536)
        self.pwm.duty_u16(duty_16)

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty_u16(0)
