#!/usr/bin/env python3

import actionlib
import json
import requests
import rospkg
import rospy
import subprocess

from cpr_indoornav_navigation_msgs.msg import *
from cpr_indoornav_navigation_msgs.srv import *

from math import pi
from math import radians as deg2rad


class MoveGoalNode:
    def __init__(self):
        rospy.init_node('indoornav_move_goal_node')

        self.hostname = rospy.get_param("~hostname", "10.252.252.1")
        self.port = rospy.get_param("~port", 5000)

        self.send_move_goal_srv = actionlib.SimpleActionServer(
            "~move_to_location", MoveToLocationAction, execute_cb=self.on_move_to_location,
            auto_start=False
        )
        self.move_to_marker_srv = actionlib.SimpleActionServer(
            "~move_to_marker", MoveToMarkerAction, execute_cb=self.on_move_to_marker,
            auto_start=False
        )
        self.execute_mission_srv = actionlib.SimpleActionServer(
            "~execute_mission", ExecuteMissionAction, execute_cb=self.on_execute_mission,
            auto_start=False
        )

    def run(self):
        get_markers_srv = rospy.Service("~get_markers", GetMarkers, self.handle_get_markers)
        get_missions_srv = rospy.Service("~get_missions", GetMissions, self.handle_get_missions)
        get_places_srv = rospy.Service("~get_places", GetPlaces, self.handle_get_places)

        self.send_move_goal_srv.start()
        self.move_to_marker_srv.start()
        self.execute_mission_srv.start()
        rospy.spin()

    def on_move_to_location(self, req):
        STATE_RUNNING = 0
        STATE_ERR = -1
        STATE_DONE = 1

        rate = rospy.Rate(1)
        try:
            rospack = rospkg.RosPack()
            script_path = f"{rospack.get_path('cpr_indoornav_navigation')}/ros2/send_move_goal.bash"
            move_goal_process = subprocess.Popen(
                ["bash", script_path, str(req.x), str(req.y), str(req.yaw)],
                stdout=subprocess.PIPE
            )
            move_goal_process.communicate()
            state = STATE_RUNNING
        except:
            state = STATE_ERR
        while state == STATE_RUNNING and not self.send_move_goal_srv.is_preempt_requested():
            rate.sleep()
            if move_goal_process.returncode is None:
                fb = MoveToLocationFeedback()
                fb.is_driving = True
                self.send_move_goal_srv.publish_feedback(fb)
                state = STATE_RUNNING
            elif move_goal_process.returncode == 0:
                rospy.logwarn("send_move_goal process reports goal reached")
                state = STATE_DONE
            else:
                rospy.logwarn(f"send_move_goal process exited with code {move_goal_process.returncode}")
                state = STATE_ERR

        result = MoveToLocationResult()
        if self.send_move_goal_srv.is_preempt_requested():
            result.reached_goal = False
            self.send_move_goal_srv.set_preempted(result, "Move goal preemted")
        elif state == STATE_DONE:
            result.reached_goal = True
            self.send_move_goal_srv.set_succeeded(result)
        else:
            result.reached_goal = False
            self.send_move_goal_srv.set_succeeded(result)

    def on_move_to_marker(self, req):
        client = actionlib.SimpleActionClient("~move_to_location", MoveToLocationAction)
        client.wait_for_server()
        goal = MoveToLocationGoal()
        goal.x = req.destination.x
        goal.y = req.destination.y
        goal.yaw = req.destination.yaw

        def on_feedback(fb):
            marker_fb = MoveToMarkerFeedback()
            marker_fb.is_driving = True
            self.move_to_marker_srv.publish_feedback(marker_fb)

        client.send_goal(goal, feedback_cb = on_feedback)
        action_finished = client.wait_for_result()
        if action_finished:
            result = client.get_result()
            marker_result = MoveToMarkerResult()
            marker_result.reached_goal = result.reached_goal
            self.move_to_marker_srv.set_succeeded(marker_result)
        else:
            self.move_to_marker_srv.set_preempted(result, "Marker goal preemted")

    def on_execute_mission(self, req):
        client = actionlib.SimpleActionClient("~move_to_marker", MoveToMarkerAction)
        client.wait_for_server()

        for t in req.mission.tasks:
            if t.place.primary_marker_intent == "WAYPOINT":
                goal = MoveToMarkerGoal()
                goal.marker = t.place.primary_marker

                fb = ExecuteMissionFeedback()
                fb.current_goal = goal.marker
                self.execute_mission_srv.publish_feedback(fb)

                client.send_goal(goal)
                action_finished = client.wait_for_result()
                if not action_finished:
                    result = ExecuteMissionResult()
                    result.mission_complete = False
                    self.execute_mission_srv.set_aborted(result, f"Sub-goal ID {goal.marker.id} did not finish")
                    return
                else:
                    action_result = client.get_result()
                    if not action_result.reached_goal:
                        result = ExecuteMissionResult()
                        result.mission_complete = False
                        self.execute_mission_srv.set_aborted(result, f"Failed to reach sub-goal ID {goal.marker.id}")
                        return
                
        result = ExecuteMissionResult()
        result.mission_complete = True
        self.execute_mission_srv.set_succeeded(result)

    def handle_get_markers(self, req):
        raw_data = self.wget_json(f"http://{self.hostname}:{self.port}/api/v2/maps/markers")
        markers = []
        for jm in raw_data["features"]:
            try:
                m = Marker()
                m.type = jm["type"]

                m.id = jm["properties"]["id"]
                m.name = jm["properties"]["name"]
                m.item_type = jm["properties"]["item_type"]
                m.marker_intent = jm["properties"]["marker_intent"]
                m.notes = jm["properties"]["notes"]

                m.x = jm["geometry"]["coordinates"][0]
                m.y = jm["geometry"]["coordinates"][1]
                m.yaw = jm["properties"]["yaw"]

                markers.append(m)
            except Exception as err:
                rospy.logwarn(f"Failed to parse marker: {err}")

        return GetMarkersResponse(markers)


    def handle_get_missions(self, req):
        places = self.handle_get_places(None).places
        places_by_id = {}
        for p in places:
            places_by_id[p.id] = p

        raw_data = self.wget_json(f"http://{self.hostname}:{self.port}/api/v2/maps/mission_templates")
        missions = []
        for jm in raw_data:
            try:
                m = Mission()
                m.name = jm["name"]
                m.id = jm["id"]
                m.description = jm["description"]
                m.tasks = []

                for jt in jm["tasks"]:
                    t = Task()
                    t.place = places_by_id[jt["place"]]
                    t.description = jt["description"]
                    t.task_type = jt["task_type"]

                    m.tasks.append(t)

                missions.append(m)
            except Exception as err:
                rospy.logwarn(f"Failed to parse mission: {err}")

        return GetMissionsResponse(missions)        

    def handle_get_places(self, req):
        markers = self.handle_get_markers(None).markers
        markers_by_id = {}
        for m in markers:
            markers_by_id[m.id] = m

        raw_data = self.wget_json(f"http://{self.hostname}:{self.port}/api/v2/maps/places")
        places = []
        for jp in raw_data:
            try:
                p = Place()
                p.name = jp["name"]
                p.id = jp["id"]
                p.place_type = jp["place_type"]
                p.primary_marker_intent = jp["primary_marker_intent"]

                if "primary_marker_id" in jp.keys():
                    p.primary_marker = markers_by_id[jp["primary_marker_id"]]

                places.append(p)
            except Exception as err:
                rospy.logwarn(f"Failed to parse place: {err}")

        return GetPlacesResponse(places)

    def wget_json(self, url):
        """Do a GET request for JSON data from the nav backpack and return the parsed JSON data
        """
        try:
            resp = requests.get(url)
            json_data = json.loads(resp.text)
            return json_data
        except Exception as err:
            rospy.logwarn(f"Failed to read {url}: err")
            return {}