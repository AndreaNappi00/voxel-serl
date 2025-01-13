import numpy as np
from typing import Tuple

from ur_env.envs.ur5_env import UR5Env
from ur_env.envs.placing_env.config import UR5PlacingCornerConfig


# used for float value comparisons (pressure of vacuum-gripper)
def is_close(value, target):
    return abs(value - target) < 1e-4


class BoxPlacingCornerEnv(UR5Env):
    def __init__(self, **kwargs):
        super().__init__(**kwargs, config=UR5PlacingCornerConfig)

    def compute_reward(self, obs, action) -> float:
        # huge action gives negative reward (like in mountain car)
        action_cost = 0.1 * np.sum(np.power(action, 2))
        action_diff_cost = 0.1 * np.sum(np.power(obs["state"]["action"] - self.last_action, 2))
        self.last_action[:] = action
        step_cost = 0.01

        suction_reward = 0.3 * float(obs["state"]["gripper_state"][1] > 0.5)
        suction_cost = 3. * float(obs["state"]["gripper_state"][1] < -0.5)

        pose = obs["state"]["tcp_pose"]
        
        orientation_cost = 1. - sum(obs["state"]["tcp_pose"][3:] * self.curr_reset_pose[3:]) ** 2
        orientation_cost = max(orientation_cost - 0.005, 0.) * 25.
        
        max_pose_diff = 0.05  # set to 5cm
        pos_diff = obs["state"]["tcp_pose"][:2] - self.goal_position[:2]
        position_cost = 10. * np.sum(
            np.where(np.abs(pos_diff) > max_pose_diff, np.abs(pos_diff - np.sign(pos_diff) * max_pose_diff), 0.0)
        )
        
        max_height_diff = 0.1  # set to 10cm
        height_diff = obs["state"]["tcp_pose"][2] - self.goal_position[2]
        position_cost += 20. * height_diff if height_diff > max_height_diff else 0.
        
        cost_info = dict(
            action_cost=action_cost,
            step_cost=step_cost,
            suction_reward=suction_reward,
            suction_cost=suction_cost,
            orientation_cost=orientation_cost,
            position_cost=position_cost,
            action_diff_cost=action_diff_cost,
            total_cost=-(-action_cost - step_cost + suction_reward - suction_cost - orientation_cost - position_cost - action_diff_cost)
        )
        for key, info in cost_info.items():
            self.cost_infos[key] = info + (0. if key not in self.cost_infos else self.cost_infos[key])
        
        if self.reached_goal_state(obs):
            self.last_action[:] = 0.
            return 100. - action_cost - orientation_cost - position_cost - action_diff_cost
        else:
            return 0. + suction_reward - action_cost - orientation_cost - position_cost - \
                suction_cost - step_cost - action_diff_cost

    def reached_goal_state(self, obs) -> bool:
        # obs[0] == gripper pressure, obs[4] == force in Z-axis
        state = obs["state"]
        goal = self.goal_position
        # return 0.1 < state['gripper_state'][0] < 0.85 and np.linalg.norm(state['tcp_pose'][:2] - goal[:2]) < 0.01 and state['tcp_pose'][2] < 0.14
        force_goal = obs["state"]["tcp_force"][0] < -7 and obs["state"]["tcp_force"][1] < -7
        height_goal = state['tcp_pose'][2] < 0.14
        gripper_goal = 0.1 < state['gripper_state'][0] < 0.85
        orientation_goal = sum(obs["state"]["tcp_pose"][3:] * self.curr_reset_pose[3:]) ** 2 > 0.9
        # if not gripper_goal:
        #     print("Gripper not closed")
        # if not force_goal:
        #     print("Force not high enough")
        # if not height_goal:
        #     print("Height not low enough")
        # print("###################################")
        # print(f"Force goal: {force_goal}, Height goal: {height_goal}, Gripper goal: {gripper_goal}")
        # print(f"force: {obs['state']['tcp_force']}")
        return gripper_goal and force_goal and height_goal