from gibson.envs.drone_env import DroneNavigateEnv
import argparse
import os
import numpy as np
from my_python_utils.common_utils import *
import matplotlib.pyplot as plt
from transforms3d import quaternions
import numpy as np

config_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'trajectory_images.yaml')
print(config_file)

trajectories_path = '/root/mounted_data/navigation_scenarios/waypoints/all'

def get_random_camera_perturbation():
    return np.array((0,0,0)), np.array((0,0,0))

def get_rotation_focused_in_point(camera_position, focus_position):
    return

camera_position_perturbation, camera_rotation_perturbation = get_random_camera_perturbation()
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default=config_file)
    args = parser.parse_args()

    env = DroneNavigateEnv(config = args.config)
    env.reset()

    empty_action = np.array([0,0,0,0,0,0,0,0])

    trajectories = {}
    json_path = os.path.join(trajectories_path, "{}.json".format(env.model_id))
    with open(json_path, "r") as f:
        trajectories = json.load(f)

    #we need 180 roll
    base_roll = quaternions.axangle2quat([0, 0, 1], 180 * np.pi / 180)
    base_pitch = quaternions.axangle2quat([1,0, 0], 90 * np.pi / 180)
    base_yaw = quaternions.axangle2quat([0, 1, 0], -10 * np.pi / 180)
    base_camera_position = np.array([0, 0, 1])

    for trajectory in trajectories[:1]:
        waypoints = trajectory['waypoints']
        for position in waypoints[:1]:

            #for k in range(10):
                base_camera_orientation = quaternions.qmult(base_yaw, quaternions.qmult(base_pitch, base_roll))
                #env.config['fov'] = 1.57 # + k*1e-1

                camera_position_perturbation, camera_rotation_perturbation = get_random_camera_perturbation()
                new_pos = (position + base_camera_position)
                new_or = base_camera_orientation
                env.robot.env.robot.reset_new_pose(new_pos, new_or)

                obs, _, _, _ = env.step(empty_action)
                v = obs["nonviz_sensor"][3]
                omega = obs["nonviz_sensor"][-1]
                imshow(obs['rgb_filled'], title='rgb')
                imshow(obs['rgb_prefilled'], title='rgb_nonfilled')

                imshow(obs['normal'], title='normal')
                imshow(obs['depth'], title='depth')
                #imshow(obs['semantics'], title='semantics')
