import numpy as np
from typing import Tuple
from scipy.spatial.transform import Rotation as R

from robotiq_env.envs.robotiq_env import RobotiqEnv
from robotiq_env.envs.basic_env.config import RobotiqCornerConfig


# used for float value comparisons (pressure of vacuum-gripper)
def is_close(value, target):
    return abs(value - target) < 1e-3

def pos_difference(quat_pose_1: np.ndarray, quat_pose_2: np.ndarray, weight: np.ndarray) -> np.ndarray:
    assert quat_pose_1.shape == (7,)
    assert quat_pose_2.shape == (7,)
    p_diff = np.sum(np.abs(quat_pose_1[:3] - quat_pose_2[:3]) * weight[:3])

    r_diff = (R.from_quat(quat_pose_1[3:]) * R.from_quat(quat_pose_2[3:]).inv()).magnitude()
    return p_diff + r_diff * np.max(weight[3:])

weight = np.array([0.7, 0.5, 1., 1, 1, 1, 1])

class RobotiqBasicEnv(RobotiqEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs, config=RobotiqCornerConfig)
        self.last_action = np.zeros(self.action_space.shape, dtype=np.float32)

    def compute_reward(self, obs, action) -> float:
        # huge action gives negative reward (like in mountain car)
        action_cost = 0.1 * np.sum(np.power(action, 2))
        step_cost = 0.01
        self.last_action[:] = action

        gripper_state = obs["state"]['gripper_state']
        suck_cost = 0.1 * float(is_close(gripper_state[0], 0.99))

        pose = obs["state"]["tcp_pose"]
        # box_xy = np.array([0.009, -0.5437])     # TODO replace with camera / pointcloud info of box
        # xy_cost = 5 * np.sum(np.power(pose[:2] - box_xy, 2))        # TODO can be ignored

        # print(f"action_cost: {action_cost}, xy_cost: {xy_cost}")
        distance_cost = np.exp(pos_difference(pose, self.config.TARGET_POSE, weight)) - 1
        # print(f"distance_cost: {distance_cost}")
        
        if self.reached_goal_state(obs):
            return 10. - action_cost - step_cost - distance_cost
            # return 10. - action_cost - step_cost - suck_cost
        else:
            return 0.0 - action_cost - step_cost - distance_cost

    def reached_goal_state(self, obs) -> bool:
        # obs[0] == gripper pressure, obs[4] == force in Z-axis
        state = obs["state"]
        self.last_action[:] = 0.
        cost = np.exp(pos_difference(state["tcp_pose"], self.config.TARGET_POSE, weight)) - 1
        print(cost)
        return cost < 0.025
        # return 0.1 < state['gripper_state'][0] < 0.85 and state['tcp_pose'][2] > 0.15  # new min height with box
