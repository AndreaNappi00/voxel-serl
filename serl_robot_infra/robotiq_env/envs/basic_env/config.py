from robotiq_env.envs.robotiq_env import DefaultEnvConfig
import numpy as np


class RobotiqCornerConfig(DefaultEnvConfig):
    # RESET_Q = np.array([[1.34231, -1.24585, 1.94961, -2.27267, -1.56428, -0.22641]])   # original one
    # RESET_Q = np.array([[1.3463, -1.3584,  1.9014, -2.1243, -1.5758, -0.2312]])
    RESET_Q = np.array([
        [3.13862559, -1.46608, 1.033933, -1.131497, -1.5641641, 0.0301942]
    ])
    RANDOM_RESET = False
    RANDOM_XY_RANGE = (0.06,)
    RANDOM_ROT_RANGE = (0.0,)
    # ABS_POSE_LIMIT_HIGH = np.array([0.14, -0.4, 0.2, 3.2, 0.1, 3.2])            # TODO euler rotations suck :/
    # ABS_POSE_LIMIT_LOW = np.array([-0.3, -0.7, -0.006, 3.0, -0.1, -3.2])
    ABS_POSE_LIMIT_HIGH = np.array([0.7, 0.3, 0.9, 3.2, 3.2, 3.2])
    ABS_POSE_LIMIT_LOW = np.array([-0.3, -0.4, 0.3, -3.2, -3.2, -3.2])
    ACTION_SCALE = np.array([0.02, 0.1, 1.], dtype=np.float32)

    ROBOT_IP: str = "192.168.1.33"
    CONTROLLER_HZ = 100
    GRIPPER_TIMEOUT = 2000  # in milliseconds
    ERROR_DELTA: float = 0.05
    FORCEMODE_DAMPING: float = 0.0  # faster
    FORCEMODE_TASK_FRAME = np.zeros(6)
    FORCEMODE_SELECTION_VECTOR = np.ones(6, dtype=np.int8)
    FORCEMODE_LIMITS = np.array([0.5, 0.5, 0.5, 1., 1., 1.])


class RobotiqCornerConfigV1(DefaultEnvConfig):
    """
    Configuration for the first set of SAC training's (only 1 box)
    """
    # RESET_Q = np.array([[1.34231, -1.24585, 1.94961, -2.27267, -1.56428, -0.22641]])   # original one
    RESET_Q = np.array([[1.3463, -1.3584, 1.9014, -2.1243, -1.5758, -0.2312]])
    # TODO make multiple reset Q positions, one for each box to train on (randomize it)
    RANDOM_RESET = False
    RANDOM_XY_RANGE = (0.06,)
    RANDOM_ROT_RANGE = (0.0,)
    ABS_POSE_LIMIT_HIGH = np.array([0.14, -0.4, 0.2, 3.2, 0.1, 3.2])
    ABS_POSE_LIMIT_LOW = np.array([-0.3, -0.7, -0.006, 3.0, -0.1, -3.2])
    ACTION_SCALE = np.array([0.02, 0.1, 1.], dtype=np.float32)

    ROBOT_IP: str = "192.168.1.33"
    CONTROLLER_HZ = 100
    GRIPPER_TIMEOUT = 2000  # in milliseconds
    ERROR_DELTA: float = 0.05
    FORCEMODE_DAMPING: float = 0.0  # faster
    FORCEMODE_TASK_FRAME = np.zeros(6)
    FORCEMODE_SELECTION_VECTOR = np.ones(6, dtype=np.int8)
    FORCEMODE_LIMITS = np.array([0.5, 0.5, 0.5, 1., 1., 1.])
