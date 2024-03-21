#!/usr/bin/env bash
eval "$(conda shell.bash hook)"
conda activate spear-env

NUM_WORKERS=30
SCENE_NAME=kujiale_0000

OMP_NUM_THREADS=4 python mujoco_export_pipeline/export_scene_to_mujoco.py -n ${NUM_WORKERS} --scene scenes/${SCENE_NAME} --rerun
