import maestro as m
import keyboard


servo = m.Controller()
servo.setAccel(0,1)
servo.setSpeed(0,1);
servo.setTarget(0,12);





def forward():
    servo.setTarget(0,1000);
    servo.setTarget(1,8000);
def backward():
    servo.setTarget(0,12);
    servo.setTarget(1,12);
def leftward():
    servo.setTarget(0,0);
    servo.setTarget(1,12);
def rightward():
    servo.setTarget(0,12);
    servo.setTarget(1,0);
def reset():
    servo.setTarget(0,0);
    servo.setTarget(1,0);

    
while True:
    
    if keyboard.is_pressed('up'):
        forward()
    if keyboard.is_pressed('down'):
        backward()
    if keyboard.is_pressed('left'):
        leftward()
    if keyboard.is_pressed('right'):
        rightward()
    if keyboard.is_pressed('space'):
        reset()
    elif keyboard.is_pressed('esc'):
        #servo.close();
        break;


