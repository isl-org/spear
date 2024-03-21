import argparse
import mujoco
import mujoco.viewer
import numpy as np
import time
import os


osp = os.path

def run_sim(filename: str, norender: bool):
    if osp.splitext(filename)[1] == '.xml':
      print('creating model from XML...')
      m = mujoco.MjModel.from_xml_path(filename)
      mjb_filename = filename.replace('xml', 'mjb')
      print('saving model in binary format...')
      mujoco.mj_saveModel(m, mjb_filename, None)
      print(f'{mjb_filename} written')
    elif osp.splitext(filename)[1] == '.mjb':
      print('creating model from MJB...')
      m = mujoco.MjModel.from_binary_path(filename)
      print('creating data...')
    else:
       raise NameError(f'invalid model filename {filename}')
    d = mujoco.MjData(m)

    if norender:
      start = time.time()
      while time.time() - start < 50:
        step_start = time.time()
        mujoco.mj_step(m, d)
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
          time.sleep(time_until_next_step)    
    else:
      act_mids = [np.mean(m.actuator(i).ctrlrange) for i in range(m.nu)]
      act_mags = [0.5*(m.actuator(i).ctrlrange[1]-m.actuator(i).ctrlrange[0]) for i in range(m.nu)]
      period = 1000

      with mujoco.viewer.launch_passive(m, d) as viewer:
        # Close the viewer automatically after 30 wall-seconds.
        start = time.time()
        t = 0
        with viewer.lock():
          viewer.scn.maxgeom = 25000
        while viewer.is_running() and time.time() - start < 300:
          step_start = time.time()

          # mj_step can be replaced with code that also evaluates
          # a policy and applies a control signal before stepping the physics.
          l = np.sin(2*np.pi*t/period)
          for i in range(m.nu):
              d.ctrl[i] = act_mids[i] + l*act_mags[i]
          mujoco.mj_step(m, d)

          # # Example modification of a viewer option: toggle contact points every two seconds.
          # with viewer.lock():
          #   viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

          # Pick up changes to the physics state, apply perturbations, update options from GUI.
          viewer.sync()

          # Rudimentary time keeping, will drift relative to wall clock.
          time_until_next_step = m.opt.timestep - (time.time() - step_start)
          if time_until_next_step > 0:
            time.sleep(time_until_next_step)    
          t += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--filename', help="path to XML or MJB file", required=True)
    parser.add_argument('--norender', action='store_true')
    args = parser.parse_args()
    run_sim(osp.expanduser(args.filename), args.norender)