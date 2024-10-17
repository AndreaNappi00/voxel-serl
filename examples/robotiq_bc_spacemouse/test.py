import rtde_control, rtde_receive
from robotiq_env.utils.vacuum_gripper import VacuumGripper
import time
import asyncio

host = "192.168.1.33"
# rtde_c = rtde_control.RTDEControlInterface(host)
rtde_r = rtde_receive.RTDEReceiveInterface(host)
gripper = VacuumGripper(host)

async def main():
    await gripper.connect()
    await gripper.activate()
    time.sleep(1)

    current_q = rtde_r.getActualQ()
    current_pose = rtde_r.getActualTCPPose()

    new_pose = [current_pose[0], current_pose[1], current_pose[2] - 0.1, current_pose[3], current_pose[4], current_pose[5]]

    # print("Current pose: ", current_pose)
    # rtde_c.moveL(new_pose, 0.25, 0.5, True)

    while True:
        print(await gripper.get_object_status())
        await asyncio.sleep(1)  # Add a sleep to avoid busy-waiting

if __name__ == "__main__":
    asyncio.run(main())
# print("Current pose: ", current_pose)
# rtde_c.moveL(new_pose, 0.25, 0.5, True)

# while True:
#     print(gripper.is_active())