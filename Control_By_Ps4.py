from pyPS4Controller.controller import Controller
import numpy as np
import cv2
from control import Control

pwmValue = 0

Control1 = Control(23, 24, 14, 15, 5, 6, 17, 27)


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
    # move forward

    def on_R3_down(self, value):
        pwmValue = int(50 + ((value / 32767) * 50))
        Control1.move_forward(pwmValue)
        print("on_R3_down: {}".format(value))

    def on_R3_up(self, value):
        pwmValue = int(((value / 32767) * 50) + 50)
        Control1.move_forward(pwmValue)
        print("on_R3_up: {}".format(value))

    # move backword

    def on_R3_right(self, value):
        pwmValue = int(50 + ((value / 32767) * 50))
        Control1.move_bakcward(pwmValue)
        print("on_R3_right: {}".format(value))

    def on_R3_left(self, value):
        pwmValue = int(((value / 32767) * 50) + 50)
        Control1.move_bakcward(pwmValue)
        print("on_R3_left: {}".format(value))
    # move right

    def on_R1_press(self):
        Control1.right(100)

    def on_R1_release(self):
        Control1.stop()
    # move left

    def on_L1_press(self):
        Control1.left(100)

    def on_L1_release(self):
        Control1.stop()


controller = MyController(interface="/dev/input/js0",
                          connecting_using_ds4drv=False)
controller.listen(timeout=60)
