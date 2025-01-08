from rtde_receive import RTDEReceiveInterface
import numpy as np

rtde_r = RTDEReceiveInterface("192.168.1.33")

print("pose: ", rtde_r.getActualQ())  # in degrees
print("position: ", rtde_r.getActualTCPPose()[:3])  # in meters