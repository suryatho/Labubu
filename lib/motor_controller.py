from machine import Pin, PWM
    
class MotorController:
    def __init__(self, pwm, in1, in2):
        self.pwm = PWM(Pin(pwm))
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        
    def set_duty_cycle(self, duty_cycle):
        self.in1.value(1 if duty_cycle > 0 else 0)
        self.in2.value(0)
        duty_16 = int((abs(duty_cycle) * 65536) / 100)
        self.pwm.duty_u16(duty_16)

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty_u16(0)
