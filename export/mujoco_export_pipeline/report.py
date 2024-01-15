import argparse 
import os
import matplotlib.pyplot as plt
from collections import deque, defaultdict
import numpy as np


osp = os.path


def report(scene_path):
    n_geoms = defaultdict(int)
    for dirname in sorted(next(os.walk(scene_path))[1]):
        dirpath = osp.join(scene_path, dirname)
        q = deque([dirpath, ])
        while len(q):
            current_dir = q.popleft()
            _, dirnames, filenames = next(os.walk(current_dir))
            if 'cvx' in current_dir:
                filenames = [f for f in filenames if osp.splitext(f)[-1] == '.obj']
                n_geoms[dirname] += len(filenames)
            else:
                q.extend([osp.join(current_dir, d) for d in dirnames])
    names = list(sorted(n_geoms.keys()))
    n_geoms_cum = np.cumsum([n_geoms[name] for name in names])
    plt.figure()
    plt.plot(n_geoms_cum)
    plt.show()
    print(f'{len(names)} objects, {n_geoms_cum[-1]} geoms')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene_path', required=True, help='relative path to the scene folder')
    args = parser.parse_args()
    report(osp.join(args.scene_path, 'output', 'Unknown'))