#!/usr/bin/env python3

import actionlib
import rospkg
import rospy
import subprocess

from cpr_indoornav_navigation_msgs.msg import *

class MoveGoalNode:
    def __init__(self):
        rospy.init_node('indoornav_move_goal_node')

        self.send_move_goal_srv = actionlib.SimpleActionServer(
            "~send_move_goal", MoveGoalAction, execute_cb=self.on_send_move_goal,
            auto_start=False
        )

    def run(self):
        self.send_move_goal_srv.start()
        rospy.spin()

    def on_send_move_goal(self, req):

        STATE_RUNNING = 0
        STATE_ERR = -1
        STATE_DONE = 1

        rate = rospy.Rate(1)
        try:
            rospack = rospkg.RosPack()
            script_path = f"{rospack.get_path('cpr_indoornav_navigation')[0]}/ros2/send_move_goal.bash"
            move_goal_process = subprocess.Popen(
                ["bash", script_path, str(req.x), str(req.y), str(req.orientation)],
                stdout=subprocess.PIPE
            )
            move_goal_process.communicate()
            state = STATE_RUNNING
        except:
            state = STATE_ERR
        while state == STATE_RUNNING and not self.send_move_goal_srv.is_preempt_requested():
            rate.sleep()
            if move_goal_process.returncode is None:
                fb = MoveGoalFeedback()
                fb.is_driving = True
                self.send_move_goal_srv.publish_feedback(fb)
                state = STATE_RUNNING
            elif move_goal_process.returncode == 0:
                rospy.logwarn("send_move_goal process reports goal reached")
                state = STATE_DONE
            else:
                rospy.logwarn(f"send_move_goal process exited with code {move_goal_process.returncode}")
                state = STATE_ERR

        result = MoveGoalResult()
        if self.send_move_goal_srv.is_preempt_requested():
            result.reached_goal = False
            self.send_move_goal_srv.set_preempted(result, "Move goal preemted")
        elif state == STATE_DONE:
            result.reached_goal = True
            self.send_move_goal_srv.set_succeeded(result)
        else:
            result.reached_goal = False
            self.send_move_goal_srv.set_succeeded(result)


            
