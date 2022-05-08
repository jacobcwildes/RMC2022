import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from communication_interface.action import Coms


class ComsActionServer(Node):

    def __init__(self):
        super().__init__('coms_action_server')
        self._action_server = ActionServer(
            self,
            Coms,
            'coms',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Coms.Feedback()
        
        if (goal_handle.request.instruction == "dump"):
            feedback_msg.navigation = "dump"
            #self.get_logger().info('Feedback: {0}'.format(feedback_msg.navigation))
            goal_handle.publish_feedback(feedback_msg)
            feedback_msg.navigation = "complete"
            goal_handle.succeed()
        elif (goal_handle.request.instruction == "dig"): 
            feedback_msg.navigation = "dig"
            #self.get_logger().info('Feedback: {0}'.format(feedback_msg.navigation))
            goal_handle.publish_feedback(feedback_msg)
            feedback_msg.navigation = "complete"
            goal_handle.succeed()
        elif (goal_handle.request.instruction == "driveToDump"): 
            #navigation to dump
            feedback_msg.navigation = "driveToDump"
            #self.get_logger().info('Feedback: {0}'.format(feedback_msg.navigation))
            goal_handle.publish_feedback(feedback_msg)
            for i in range(5):
                if (i <= 3):
                    feedback_msg.navigation = "left" #This is just the direction or whatever for nav
                    #self.get_logger().info('Feedback: {0}'.format(feedback_msg.navigation))
                    goal_handle.publish_feedback(feedback_msg)
                else:
                    feedback_msg.navigation = "complete"
                    goal_handle.succeed()
                time.sleep(1)	
        elif (goal_handle.request.instruction == "driveToDig"): 
            #navigation to drop
            feedback_msg.navigation = "driveToDig"
            #self.get_logger().info('Feedback: {0}'.format(feedback_msg.navigation))
            goal_handle.publish_feedback(feedback_msg)
            for i in range(5):
                if (i <= 3):
                    feedback_msg.navigation = "right" #This is just the direction or whatever for nav
                    #self.get_logger().info('Feedback: {0}'.format(feedback_msg.navigation))
                    goal_handle.publish_feedback(feedback_msg)
                else:
                    feedback_msg.navigation = "complete"
                    goal_handle.succeed()
                time.sleep(1)	

        result = Coms.Result()

        result.completion = feedback_msg.navigation

        return result


def main(args=None):
    rclpy.init(args=args)

    coms_action_server = ComsActionServer()

    rclpy.spin(coms_action_server)


if __name__ == '__main__':
    main()
