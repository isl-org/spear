import numpy as np
import transforms3d.quaternions as txq


def name_in_names(name, names):
  for n in names:
      if name in n:
          return n
  return None


def xyzquat_to_T(xyz, quat):
    T = np.eye(4)
    T[:3, :3] = txq.quat2mat(quat)
    T[:3,  3] = xyz
    return T


def pose_rh_to_lh(xyz, quat):
    return (xyz[0], -xyz[1], xyz[2]), (quat[0], quat[1], -quat[2], quat[3])


def T_to_xyzquat(T):
    return T[:3, 3].tolist(), txq.mat2quat(T[:3, :3]).tolist()