import subprocess
from tqdm import tqdm
import os

#execute as:
#blender -b --python visualize_path.py
trajectories_path = '/data/vision/torralba/datasets/gibson/navigation_scenarios/waypoints/all'
envs = [k.replace('.json', '') for k in os.listdir(trajectories_path)]
for env in tqdm(envs):
  try:
    text_file = open("model.txt", "w")
    text_file.write(env)
    text_file.close()
    subprocess.call('blender -b --python visualize_path.py', shell=True)
  except Exception as e:
    import traceback

    traceback.print_exc()