from gymnasium.envs.registration import register
import numpy as np

register(
    id="box_picking_basic_env",
    entry_point="ur_env.envs.basic_env:BoxPickingBasicEnv",
    max_episode_steps=500,
)

register(
    id="box_placing_corner_env",
    entry_point="ur_env.envs.placing_env:BoxPlacingCornerEnv",
    max_episode_steps=500,
)

register(
    id="box_picking_camera_env",
    entry_point="ur_env.envs.camera_env:UR5CameraEnv",
    max_episode_steps=100,
)

register(
    id="box_picking_camera_env_tests",
    entry_point="ur_env.envs.camera_env:UR5CameraEnvTest",
    max_episode_steps=100,
)

register(
    id="box_picking_camera_env_eval",
    entry_point="ur_env.envs.camera_env:UR5CameraEnvEval",
    max_episode_steps=100,
)

register(
    id="box_picking_camera_env_demo",
    entry_point="ur_env.envs.camera_env:UR5CameraEnvDemo",
    max_episode_steps=100,
)

