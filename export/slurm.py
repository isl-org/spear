#!/usr/bin/env python
import os
import sys
import random
import string
import argparse

osp = os.path


def random_filename(N=5):
    return ''.join(random.SystemRandom().choice(string.ascii_lowercase + string.digits) for _ in range(N))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', choices=('cpu', 'gpu', 'g24', 'g48'), default='cpu')
    parser.add_argument('-c', default=2, type=int)
    parser.add_argument('-g', default=0, type=int)
    parser.add_argument('-w', default=None)
    parser.add_argument('--qos', default='normal')
    parser.add_argument('--conda_env_name', default='mujoco_export_pipeline')
    parser.add_argument('--log_dir', default=osp.join('logs', 'slurm'))
    parser.add_argument('--array', default=None)
    parser.add_argument('cmd', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    cmd = ' '.join(args.cmd)
    print(cmd)

    if args.c < 2:
      print(f'-c must be >=2, it is {args.c}')
      sys.exit(-1)
  
    log_dir = osp.expanduser(args.log_dir)
    if not osp.isdir(log_dir):
        os.mkdir(log_dir)
    name = osp.join(log_dir, random_filename())
    job_filename = '{:s}.slurm'.format(name)
    with open(job_filename, 'w') as fh:
        fh.writelines("#!/bin/bash\n")
        fh.writelines("#SBATCH --output={:s}_%A_%a.out\n".format(name))
        print('stdout in {:s}.out'.format(name))
        fh.writelines("#SBATCH --error={:s}_%A_%a.err\n".format(name))
        print('stderr in {:s}.err'.format(name))
        # actually we have multiple tasks but they need different #cpus, so...
        fh.writelines(f'#SBATCH --ntasks=1\n')
        fh.writelines(f'#SBATCH --nodes=1\n')
        fh.writelines('#SBATCH -p {:s}\n'.format(args.p))
        fh.writelines('#SBATCH -c {:d}\n'.format(args.c))
        if args.w is not None:
          print('Requesting specifically node {:s}'.format(args.w))
          fh.writelines('#SBATCH -w {:s}\n'.format(args.w))
        fh.writelines('#SBATCH --qos {:s}\n'.format(args.qos))
        if args.g > 0:
            fh.writelines('#SBATCH --gres=gpu:{:d}\n'.format(args.g))
        # fh.writelines('#SBATCH --get-user-env\n')
        fh.writelines('eval "$(conda shell.bash hook)"\n')
        fh.writelines('conda activate ~/miniconda3/envs/{:s}\n'.format(args.conda_env_name))
        fh.writelines('{:s}\n'.format(cmd))
        fh.writelines('exit\n')
    sb_cmd = 'sbatch'
    if args.array is not None:
        sb_cmd = f'{sb_cmd} --array={args.array}'
    sb_cmd = f'{sb_cmd} {job_filename}'
    print('slurm command is: ', sb_cmd)
    os.system(sb_cmd)
