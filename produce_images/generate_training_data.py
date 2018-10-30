from gibson.envs.drone_env import DroneNavigateEnv
import argparse
import os
import numpy as np
from my_python_utils.common_utils import *
import matplotlib.pyplot as plt
from transforms3d import quaternions
import numpy as np
import yaml
import time
from tqdm import tqdm
import json
import subprocess
config_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'trajectory_images.yaml')

trajectories_path = '/root/mounted_data/navigation_scenarios/waypoints/all'
available_environments = [k.replace('.json', '') for k in os.listdir(trajectories_path)]
out_dir = '/root/mounted_output'

def get_random_camera_perturbation(base_camera_position, base_roll, base_pitch, base_yaw, no_perturbation = False):
    if no_perturbation:
        random_pitch_perturbation = 0
        random_yaw_perturbation = 0
        random_height_perturbation = 0
    else:
        random_pitch_perturbation = np.random.uniform(-180,180)
        random_yaw_perturbation = np.random.uniform(-30,30)
        random_height_perturbation = np.random.uniform(-0.5,0.5)

    base_roll = quaternions.axangle2quat([0, 0, 1], base_roll * np.pi / 180)
    base_pitch = quaternions.axangle2quat([1, 0, 0], (base_pitch + random_pitch_perturbation)* np.pi / 180)
    base_yaw = quaternions.axangle2quat([0, 1, 0], (base_yaw  + random_yaw_perturbation)* np.pi / 180)
    base_camera_orientation = quaternions.qmult(base_yaw, quaternions.qmult(base_pitch, base_roll))

    return base_camera_position + np.array((0,random_height_perturbation, 0)), base_camera_orientation

def get_rotation_focused_in_point(camera_position, focus_position):
    return

with open(config_file, 'r') as f:
    config_data = yaml.load(f)
if __name__ == '__main__':
    for actual_env in tqdm(available_environments):
        if 'house' in actual_env:
            # we dont have trajectories for matterport envs
            continue
        subprocess.call('pkill -f depth_render', shell=True)
        config_data['model_id'] = actual_env
        env_dir = out_dir + '/output_focal_modified/{}'.format(actual_env)
        if os.path.exists(env_dir):
            folders = [ k for k in os.listdir(env_dir) if 'traj' in k]
            if len(folders) == 100:
                continue
        try:
            env = DroneNavigateEnv(config=config_data)
            env.sensor_size = 0.035
            env.focal = 0.035

            env.reset()

            empty_action = np.array([0,0,0,0,0,0,0,0])

            trajectories = {}
            json_path = os.path.join(trajectories_path, "{}.json".format(actual_env))
            with open(json_path, "r") as f:
                trajectories = json.load(f)

            #we need 180 roll
            base_roll = 180
            base_pitch = 0
            base_yaw = 0
            base_camera_position = np.array([0, 0, 0])
            obs = list()
            then = time.time()
            for i_traj in tqdm(range(len(trajectories))):
                i_traj = 2
                trajectory = trajectories[i_traj]
                waypoints = trajectory['waypoints']
                poses = list()
                folder = env_dir + '/traj_{}/'.format(str(i_traj).zfill(4))
                for i_waypoint in range(len(waypoints)):
                    position = waypoints[1]

                    camera_position_perturbation, camera_rotation_perturbed = get_random_camera_perturbation(base_camera_position, base_roll, base_pitch, base_yaw, no_perturbation = True)
                    new_pos = (position + base_camera_position + camera_position_perturbation)
                    new_or = camera_rotation_perturbed
                    env.robot.env.robot.reset_new_pose(new_pos, new_or)
                    env.focal = (i_waypoint + 1)*0.002 + 0.035
                    env.sensor_size = 0.035

                    obs, _, _, _ = env.step(empty_action)
                    imshow(obs['depth'], env='gibson_test', title='render_depth')
                    imshow(obs['rgb_prefilled'], env='gibson_test', title='rendered_image')

                    poses.append(np.concatenate((new_pos, new_or)).tolist())
                    if not os.path.exists(folder):
                        os.makedirs(folder)
                    cv2.imwrite(folder + '/{}.jpg'.format(str(i_waypoint).zfill(4)), obs['rgb_prefilled'])
                    cv2.imwrite(folder + '/normal_{}.jpg'.format(str(i_waypoint).zfill(4)), obs['normal'])
                    depth = obs['depth']
                    #TODO: probably with quantiles to avoid outliers
                    max_depth = depth.max()
                    min_depth = depth.min()
                    normalized_depth = (depth - min_depth)/(max_depth - min_depth)
                    normalized_depth = np.array(255.0*normalized_depth, dtype='uint8')
                    cv2.imwrite(folder + '/depth_{}_max_{:.2f}_min_{:.2f}.jpg'.format(str(i_waypoint).zfill(4), max_depth, min_depth), normalized_depth)

                    #for k in obs.keys():
                    #    if k in ['rgb_filled', 'depth', 'normal', 'rgb_prefilled']:
                    #        imshow(obs[k], title=k)
                        #v = obs["nonviz_sensor"][3]
                        #omega = obs["nonviz_sensor"][-1]

                        #imshow(obs['rgb_filled'], title='rgb')
                        #imshow(obs['rgb_prefilled'], title='rgb_nonfilled')

                        #imshow(obs['normal'], title='normal')
                        #imshow(obs['depth'], title='depth')
                        #imshow(obs['semantics'], title='semantics')
                        #obs.append(obs['rgb_filled'])
                with open(folder + '/poses.json', 'w') as outfile:
                    json.dump(poses, outfile)
            env.close()
        except Exception as e:
            print(e)
            try:
                env.close()
            except:
                pass
            continue
        subprocess.call('pkill -f depth_render', shell=True)