import asyncio
import agility
import agility.messages as msgs


def error_handler(msg):
    # Message handler that prints out error/warning messages
    print(msg.type, msg.info)
    return True


# Main function, starts all other tasks
async def main():
    # Connect to simulator on localhost:8080 by default
    async with agility.JsonApi() as api:
        # Install default error/warning handlers that just print messages
        api.handle('error', error_handler)
        api.handle('warn', error_handler)

        # Request the change action command privilege
        await api.request_privilege('change-action-command')

        # Start printing out timestamped position data for the robot
        # monitor_task = asyncio.create_task(print_position(api)) # Python 3.7+
        monitor_task = asyncio.ensure_future(print_position(api))

        # Command the robot to walk forward three meters relative to its
        # current position
        obj = await api.get_response(msgs.AddObject(
            transform={},
            relative_to={'robot-frame': msgs.RobotFrame.BASE},
            stored_relative_to={'special-frame': 'world'},
        ))
        await api.wait_action(msgs.ActionGoto(
            target={'xy': [3, 0]},
            reference_frame={'object-id': obj.id}
        ))


# Task for monitoring robot position
async def print_position(api):
    # Get timestamped robot frame position data
    queries = [
        msgs.GetTimestamp(),
        msgs.GetObjectKinematics(object={'robot-frame': msgs.RobotFrame.BASE}),
    ]

    # Send a periodic query group, clean up query when task is canceled
    async with api.periodic_query(queries, 0.1) as query:
        # When each response is received, print a message to the console
        async for result in query:
            print(f'time: {result[0].run_time}, '
                  f'x: {result[1].transform.rpyxyz[3]}')

# Launch simulator
# Change this to the path to the simulator binary on your system
sim_path = './ar-control'
with agility.Simulator(sim_path) as sim:
    # Start running main function
    # asyncio.run(main()) # Python 3.7+
    asyncio.get_event_loop().run_until_complete(main())
