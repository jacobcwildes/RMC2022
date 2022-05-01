import maestro as m
import time

#fluid vars
maxSpeed = 0
direction = ""
readyForInstructions = True
instruction = ""

#static speed vars
base = 6000
full = 1000
fine = 200
boom = 1000
chain = 2000
dump = 500

#intial instructions
reset()
servo = m.Controller()

#----------------
#motor map      |
#----------------
#right drive = 0|
#leftdrive = 1  |
#boom = 2       |
##chain = 3     |
#dumper = 4     |
#----------------

#main loop-----------------------------------------------------------

while (readyForInstructions):
	readInstructions()
	if (instruction == "goToDump") driveTo("dump")
	elif (instruction == "goToDrop") driveTo("dump")
	elif (instruction == "dump") dump()
	elif (instruction == "dig") dig()
	
#major function-------------------------------------------------------	

def driveTo(location):
	startAction()
	setFast();
	#This part will be temp until lidar works
	navigate(location);
	#-----
	reset()
	doneAction()
		
def dump():
	startAction()
	alignWithDropOff()
	dumperDump()
	time.sleep(5)
	dumperReturn()
	time.sleep(5)
	reset()
	doneAction()
	
def dig():
	startAction()
	boomDown()
	time.sleep(2)
	reset()
	chainGo()
	time.sleep(15)
	chainStop()
	boomUp()
	time.sleep(2)
	reset()
	doneAction()
	
#minor movement functions-----------------------------------------------------

def alignWithDropOff():
	setSlow()
	#just temp until lidar works
	left()
	time.sleep(1)
	right()
	time.sleep(1)
	reset()	
	
def advance():
	if(direction = "dig"):
		forward()
	if(direction == "drop"):
		backward()

def navigate(dir):
	direction = dir
	advance()
	time.sleep(1)
	left()
	time.sleep(1)
	advance()
	time.sleep(1)
	right()
	time.sleep(1)
	advance()
	time.sleep(1)
	reset()
	
#setters and getters -------------------------------------------------------

def readInstructions():
	#temp unitl ros is set up
	#somehow grab listeners 
	intructions = "goToDump"

def startAction():
	readyForInstructions = False
	
def doneAction():
	readyForInstructions = True	

def setSlow():
	setSpeed("fine")
def setFast():
	setSpeed("full")

def setSpeed(degree):
	if (degree == "fine"):
		servo.setAccel(0,0)
		servo.setSpeed(1,1)
		maxSpeed = fine
	elif (degree == "full"):
		servo.setAccel(0,0)
		servo.setSpeed(1,0)
		maxSpeed = full
	
#motor setters -----------------------------------------------------------	

def forward():
    servo.setTarget(0,base - maxSpeed);
    servo.setTarget(1,base + maxSpeed);  
def backward():
    servo.setTarget(0,base + maxSpeed);
    servo.setTarget(1,base - maxSpeed);
def leftward():
    servo.setTarget(1,base - maxSpeed);
    servo.setTarget(0,base - maxSpeed);
def rightward():
    servo.setTarget(0,base + maxSpeed);
    servo.setTarget(1,base + maxSpeed);
def boomUp():
    servo.setTarget(2,base-boom);
def boomDown():
    servo.setTarget(2,base+boom);
def chainGo():
    servo.setTarget(3,base-chain);
def chainStop():
    servo.setTarget(3,base);    
def brake():
    servo.setTarget(0,base);
    servo.setTarget(1,base);
def dumperDump():
    servo.setTarget(4,base-dump);
def dumperReturn():
    servo.setTarget(4,base+dump);    
def reset():
    servo.setTarget(0,base);
    servo.setTarget(1,base);
    servo.setTarget(2,base);
    servo.setTarget(3,base);
    servo.setTarget(4,base);

