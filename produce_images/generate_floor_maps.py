from my_python_utils.common_utils import *
from tqdm import tqdm
import subprocess
config_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'trajectory_images.yaml')

trajectories_path = '/data/vision/torralba/datasets/gibson/navigation_scenarios/waypoints/all'
available_environments = [k.replace('.json', '') for k in os.listdir(trajectories_path)]
out_dir = '/data/vision/torralba/scratch2/mbaradad/movies_semantic/output_gibson/floor_plans'
dataset_path = '/data/vision/torralba/datasets/gibson/dataset/'
if __name__ == '__main__':
    for actual_env in tqdm(available_environments):
        #subprocess.call('pkill -f depth_render', shell=True)
        trajectory_path = trajectories_path + '/' + actual_env
        "blender -b --python visualize_path.py --filepath {} --datapath {} --renderpath {} --model  {} --idx 1 ".format(trajectory_path, dataset_path, out_dir, actual_env)



