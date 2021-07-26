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
    
    @file   action_stand.py
    @author Grant Gibson (Biped Robotics Lab, University of Michigan)
    @brief  Send 'action-stand' command to Digit with JSON API 
 '''
 
import sys
import rospy
import asyncio
import agility
import agility.messages as msgs

# Main function, starts all other tasks
async def main(ip_address):
    # Connect to simulator on localhost:8080 by default
    async with agility.JsonApi(address=ip_address) as api:
        # Request the change action command privilege
        await api.request_privilege('change-action-command')

        # Command the robot to walk forward three meters relative to its
        # current position
        await api.wait_action('action-stand')
            
if __name__ == "__main__":
    # Identify experiment mode
    exp_mode = rospy.get_param('action_stand_node/exp_mode')
    if exp_mode == 'sim':
        ip_address = '127.0.0.1'
    elif exp_mode == 'real':
        ip_address = '10.10.1.1'
    else:
        print("!!!!!!!! ERROR LOADING IP ADDRESS !!!!!!!!!")
    
    # Start running main function
    # asyncio.run(main()) # Python 3.7+
    asyncio.get_event_loop().run_until_complete(main(ip_address))