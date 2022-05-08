import maestro as m
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from communication_interface.action import Coms

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

#fluid vars
maxSpeed = 0
direction = ""
readyForInstructions = True
client_instructions = "driveToDig"

#static speed vars
base = 6000
full = 1000
fine = 200
boom = 1000
chain = 2000
dump = 500

#major function-------------------------------------------------------			
def dump():
        print("dump")
        alignWithDropOff()
        dumperDump()
        time.sleep(5)
        dumperReturn()
        time.sleep(5)
        reset()

def dig():
        print("dig")
        boomDown()
        time.sleep(2)
        reset()
        chainGo()
        time.sleep(15)
        chainStop()
        boomUp()
        time.sleep(2)
        reset()

#minor movement functions-----------------------------------------------------

def alignWithDropOff():
        setSlow()
        #just temp until lidar works
        leftward()
        time.sleep(1)
        rightward()
        time.sleep(1)
        reset()	

def advance():
        if(direction == "dig"):
                forward()
        if(direction == "dump"):
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

def setSlow():
        setSpeed("fine")
def setFast():
        setSpeed("full")

def setSpeed(degree):
        if (degree == "fine"):
                print("setFine")
                servo.setAccel(0,0)
                servo.setSpeed(1,1)
                maxSpeed = fine
        elif (degree == "full"):
                print("setFull")
                servo.setAccel(0,0)
                servo.setSpeed(1,0)
                maxSpeed = full

#motor setters -----------------------------------------------------------	

def forward():
    print("forward")
    servo.setTarget(0,base - maxSpeed);
    servo.setTarget(1,base + maxSpeed);  
def backward():
    print("backward")
    servo.setTarget(0,base + maxSpeed);
    servo.setTarget(1,base - maxSpeed);
def leftward():
    print("leftward")
    servo.setTarget(1,base - maxSpeed);
    servo.setTarget(0,base - maxSpeed);
def rightward():
    print("rightward")
    servo.setTarget(0,base + maxSpeed);
    servo.setTarget(1,base + maxSpeed);
def boomUp():
    print("boomUp")
    servo.setTarget(2,base-boom);
def boomDown():
    print("boomDown")
    servo.setTarget(2,base+boom);
def chainGo():
    print("chainGo")
    servo.setTarget(3,base-chain);
def chainStop():
    print("chainStop")
    servo.setTarget(3,base);    
def brake():
    print("brake")
    servo.setTarget(0,base);
    servo.setTarget(1,base);
def dumperDump():
    print("dumperDump")
    servo.setTarget(4,base-dump);
def dumperReturn():
    print("dumperReturn")
    servo.setTarget(4,base+dump);    
def reset():
    print("reset")
    servo.setTarget(0,base);
    servo.setTarget(1,base);
    servo.setTarget(2,base);
    servo.setTarget(3,base);
    servo.setTarget(4,base);


#-----------------------------------------------------------------

class ComsActionClient(Node):

    def __init__(self):
        super().__init__('coms_action_client')
        self._action_client = ActionClient(self, Coms, 'coms')

    def send_goal(self, instruction):
        goal_msg = Coms.Goal()
        goal_msg.instruction = instruction
        print(goal_msg.instruction)

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.completion))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.navigation))
        #Now use feedback to do function
        navi = feedback.navigation
        if(navi == "dump"): dump()
        elif(navi == "dig"): dig()
        elif(navi == "driveToDig"): 
                setFast();
                print("setdirdig")
                direction = "dig"
        elif(navi == "driveToDump"): 
                setFast();
                print("setdirdump")
                direction = "dump"
        elif(navi == "advance"): 
                advance()
                time.sleep(1)
                brake()
        elif(navi == "left"): 
                leftward()
                time.sleep(1)
                brake()
        elif(navi == "right"): 
                rightward()	
                time.sleep(1)
                brake()
        reset()

def main(args=None):
    reset()
    client_instructions = "driveToDig"
    
    while (True):
        rclpy.init(args=args)

        action_client = ComsActionClient()
        
        print("Sending Goal...")
        
        action_client.send_goal(client_instructions)

        rclpy.spin(action_client)
    
        print("Finished Executing...")
        
        time.sleep(5)
        
        if(client_instructions == "driveToDig") : client_instructions = "dig"
        elif(client_instructions == "dig") : client_instructions = "driveToDump"
        elif(client_instructions == "driveToDump") : client_instructions = "dump"
        elif(client_instructions == "dump") : client_instructions = "driveToDig"
       
    


if __name__ == '__main__':
    main()