#!/usr/bin/env python3

'''
    Copyright (C) July 2021, Grant Gibson (Biped Robotics Lab, University of Michigan)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
    
    @file   query_kinematics.py
    @author Grant Gibson (Biped Robotics Lab, University of Michigan)
    @brief  Query Kinematic information with JSON API and publish base_pose ROS topic
 '''

import rospy
from std_msgs.msg import Float32MultiArray
import asyncio
import agility
import agility.messages as msgs


async def main(ip_address):
    pub = rospy.Publisher('base_pose', Float32MultiArray, queue_size=10)
    rospy.init_node('query_kinematics_node', anonymous=True)
    # rate = rospy.Rate(10)  # 10hz

    async with agility.JsonApi(address=ip_address) as api:

        # Get responses for a group of simultaneous queries
        # The list can mix and match different ways of specifying messages
        # queries = ["get-timestamp"]
        queries = [
            "get-timestamp",
            ["get-object",
                {"object": {"special-frame": "world"}}],
            ["get-object",
                {"object": {"robot-frame": "base"}}],
            ["get-object-kinematics",
                {"object": {"robot-frame": "base"}, "relative-to": {"special-frame": "world"}}],
            ["get-object",
                {"object": {"robot-frame": "upper-velodyne-vlp16"}}],
            ["get-object-kinematics", {"object": {
                "robot-frame": "upper-velodyne-vlp16"}, "relative-to": {"robot-frame": "base"}}],
            ["get-object",
                {"object": {"robot-frame": "forward-chest-realsense-d435"}}],
            ["get-object-kinematics",
                {"object": {"robot-frame": "forward-chest-realsense-d435"}, "relative-to": {"robot-frame": "base"}}],
            ["get-object",
                {"object": {"robot-frame": "forward-pelvis-realsense-d430"}}],
           ["get-object-kinematics", 
                {"object": {"robot-frame": "forward-pelvis-realsense-d430"}, "relative-to": {"robot-frame": "base"}}],
           ["get-object",
               {"object": {"robot-frame": "downward-pelvis-realsense-d430"}}],
           ["get-object-kinematics", {"object": {
               "robot-frame": "downward-pelvis-realsense-d430"}, "relative-to": {"robot-frame": "base"}}],
           ["get-object",
               {"object": {"robot-frame": "backward-pelvis-realsense-d430"}}],
           ["get-object-kinematics",
                {"object": {"robot-frame": "backward-pelvis-realsense-d430"}, "relative-to": {"robot-frame": "base"}}]
        ]
        
        query_period = 0.1 # seconds
        async with api.periodic_query(queries, query_period) as q:
            async for results in q:
                rospy.loginfo(f"run-time: {results[0].run_time}, "
                              f"unix-time: {results[0].unix_time}, \n \n"
                              f"name: {results[1].attributes.name}, "
                              f"id: {results[1].id}, \n \n"
                              f"name: {results[2].attributes.name}, "
                              f"id: {results[2].id}, "
                              f"relative to: {results[3].parent}, \n"
                              f"pose (rpyxyz): {results[3].transform.rpyxyz}, \n"
                              f"velocity: {results[3].velocity.rpyxyz} \n \n"
                              f"name: {results[4].attributes.name}, "
                              f"id: {results[4].id}, "
                              f"relative to: {results[5].parent}, \n"
                              f"pose (rpyxyz): {results[5].transform.rpyxyz}, \n \n"
                              f"name: {results[6].attributes.name}, "
                              f"id: {results[6].id}, "
                              f"relative to: {results[7].parent}, \n"
                              f"pose (rpyxyz): {results[7].transform.rpyxyz}, \n \n"
                              f"name: {results[8].attributes.name}, "
                              f"id: {results[8].id}, "
                              f"relative to: {results[9].parent}, \n"
                              f"pose (rpyxyz): {results[9].transform.rpyxyz}, \n \n"
                              f"name: {results[10].attributes.name}, "
                              f"id: {results[10].id}, "
                              f"relative to: {results[11].parent}, \n"
                              f"pose (rpyxyz): {results[11].transform.rpyxyz}, \n \n"
                              f"name: {results[12].attributes.name}, "
                              f"id: {results[12].id}, "
                              f"relative to: {results[13].parent}, \n"
                              f"pose (rpyxyz): {results[13].transform.rpyxyz}, \n \n")
                pose = Float32MultiArray()
                pose.data = results[3].transform.rpyxyz
                pub.publish(pose)
                print("=====================================")

if __name__ == "__main__":

    exp_mode = rospy.get_param('query_kinematics_node/exp_mode')
    if exp_mode == 'sim':
        ip_address = '127.0.0.1'
    elif exp_mode == 'real':
        ip_address = '10.10.1.1'
    else:
        print("!!!!!!!! ERROR LOADING IP ADDRESS !!!!!!!!!")


    print("starting...")
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main(ip_address))
    print("finished")
