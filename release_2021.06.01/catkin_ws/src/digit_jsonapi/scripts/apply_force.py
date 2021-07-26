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
    
    @file   apply_force.py
    @author Grant Gibson (Biped Robotics Lab, University of Michigan)
    @brief  Apply force to Digit in simulator with JSON API
 '''
 
import sys
import rospy
import asyncio
import agility
import agility.messages as msgs

# Main function, starts all other tasks
async def main(mx,my,mz,fx,fy,fz,duration):
    # Connect to simulator on localhost:8080 by default
    async with agility.JsonApi() as api:
        # Request the change action command privilege
        await api.request_privilege('change-action-command')

        # Command the robot to walk forward three meters relative to its
        # current position
        await api.wait_action(['simulator-apply-force',{'model-id': 0,'offset': [0,0,0],'reference':'body','force':{'rpyxyz':[mx,my,mz,fx,fy,fz]},'duration': duration}])
    
if __name__ == "__main__":
    
    # Extract Wrench Parameters
    mx = rospy.get_param('apply_force_node/mx')
    my = rospy.get_param('apply_force_node/my')
    mz = rospy.get_param('apply_force_node/mz')
    fx = rospy.get_param('apply_force_node/fx')
    fy = rospy.get_param('apply_force_node/fy')
    fz = rospy.get_param('apply_force_node/fz')
    
    # Extract duration
    duration = rospy.get_param('apply_force_node/duration')
    
    # Printx
    print("mx = ",mx,
        "\nmy = ",my,
        "\nmz = ",mz,
        "\nfx = ",fx,
        "\nfy = ",fy,
        "\nfz = ",fz,
        "\nduration = ",duration)
    
    # Start running main function
    # asyncio.run(main()) # Python 3.7+
    asyncio.get_event_loop().run_until_complete(main(mx,my,mz,fx,fy,fz,duration))