from time import sleep
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


class Control:
    def __init__(self, RPWM, LPWM, L_EN, R_EN, RPWM_2, LPWM_2, L_EN_2, R_EN_2):
        # left motors
        self.RPWM = RPWM
        self.LPWM = LPWM
        self.L_EN = L_EN
        self.R_EN = R_EN

        # right motors
        self.RPWM_2 = RPWM_2
        self.LPWM_2 = LPWM_2
        self.L_EN_2 = L_EN_2
        self.R_EN_2 = R_EN_2

        # output pins
        GPIO.setup(self.RPWM, GPIO.OUT)
        GPIO.setup(self.LPWM, GPIO.OUT)
        GPIO.setup(self.L_EN, GPIO.OUT)
        GPIO.setup(self.R_EN, GPIO.OUT)
        GPIO.output(self.L_EN, True)
        GPIO.output(self.R_EN, True)
        GPIO.setup(self.RPWM_2, GPIO.OUT)
        GPIO.setup(self.LPWM_2, GPIO.OUT)
        GPIO.setup(self.L_EN_2, GPIO.OUT)
        GPIO.setup(self.R_EN_2, GPIO.OUT)
        GPIO.output(self.L_EN_2, True)
        GPIO.output(self.R_EN_2, True)

        # PWM
        self.rpwm = GPIO.PWM(self.RPWM, 100)
        self.lpwm = GPIO.PWM(self.LPWM, 100)
        self.rpwm_2 = GPIO.PWM(self.RPWM_2, 100)
        self.lpwm_2 = GPIO.PWM(self.LPWM_2, 100)

        # initialize the PWM to zero
        self.rpwm.ChangeDutyCycle(0)
        self.rpwm.start(0)
        self.rpwm_2.ChangeDutyCycle(0)
        self.rpwm_2.start(0)
        self.lpwm.ChangeDutyCycle(0)
        self.lpwm.start(0)
        self.lpwm_2.ChangeDutyCycle(0)
        self.lpwm_2.start(0)
    # Function to stop all motors

    def stop(self):
        self.rpwm.ChangeDutyCycle(0)
        self.rpwm_2.ChangeDutyCycle(0)
        self.lpwm.ChangeDutyCycle(0)
        self.lpwm_2.ChangeDutyCycle(0)
    # Function to move forward

    def move_forward(self, value):
        self.value = value
        Control.stop(self)
        self.rpwm.start(0)
        self.rpwm.ChangeDutyCycle(self.value)
        self.rpwm_2.start(0)
        self.rpwm_2.ChangeDutyCycle(self.value) 

    # Function to move bakcward
    def move_bakcward(self, value):
        self.value = value
        Control.stop(self)
        self.lpwm.start(0)
        self.lpwm.ChangeDutyCycle(self.value)
        self.lpwm_2.start(0)
        self.lpwm_2.ChangeDutyCycle(self.value)

    # Function to move right
    def left(self, value):
        self.value = value
        Control.stop(self)
        self.lpwm.start(0)
        self.lpwm.ChangeDutyCycle(self.value)
        self.rpwm_2.start(0)
        self.rpwm_2.ChangeDutyCycle(self.value)

    # Function to move left
    def right(self, value):
        self.value = value
        Control.stop(self)
        self.rpwm.start(0)
        self.rpwm.ChangeDutyCycle(self.value)
        self.lpwm_2.start(0)
        self.lpwm_2.ChangeDutyCycle(self.value)
