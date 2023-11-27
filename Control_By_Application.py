from bluedot.btcomm import BluetoothServer
from signal import pause
from control import Control

pwmValue = 0

Control1 = Control(23, 24, 14, 15, 5, 6, 17, 27)
value_move = 0
value_dirction = 0
def data_received(data):
    print(data)
    try:
        if(data == '0'):
            Control1.move_forward(0)
        if(data[0] == '1' and len(data) > 3):
            if(float(data[1:]) < 0):
                print("back")
                value_move = abs(float(data[1:])) * 100
                print(value_move)
                Control1.move_bakcward(value_move)
            elif(float(data[1:]) > 0):
                print("forward")
                value_move = float(data[1:]) * 100
                print(value_move)
                Control1.move_forward(value_move)
        if(data[0] == '2' and len(data) > 3):
            if(float(data[1:]) < 0):
                print("left")
                value_dirction = abs(float(data[1:])) * 100
                print(value_dirction)
                Control1.left(value_dirction)
            elif(float(data[1:]) > 0):
                print("right")
                value_dirction = abs(float(data[1:])) * 100
                print(value_dirction)
                Control1.right(value_dirction)
        

        # print(data[1:])
    except ValueError:
        Control1.move_forward(0)
        print('error')


s = BluetoothServer(data_received)
pause()
