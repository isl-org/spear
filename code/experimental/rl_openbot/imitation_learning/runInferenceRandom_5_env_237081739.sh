#!/bin/bash

#SBATCH -p g24
#SBATCH --gres=gpu:1
#SBATCH -c 14

source /home/qleboute/anaconda3/bin/activate interiorsim-env
python run.py -i 1000 -r 100 -s "Infer" -m "237081739" -c 1 -p "5_envs"
