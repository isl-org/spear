"""
Run this script from UE
Even though we eventually would like to run physics in a left-handed coordinate system to match UE, this script
exports everything in the right-handed coordinate system. This is done to maintain script-level consistency, because
GLTF export (implemented by UE) always exports objects in a right-handed coordinate system. The unit of translation 
can be set, however. It affects both the offsets and the geometry. So we set that to cm, overriding the default
GLTF exporter behaviour of expressing translation in m. Unit of rotation (joint limits and references) is degrees.
"""

import argparse
from collections import namedtuple
import json
import os
import unreal
import time


osp = os.path
ss = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world = ss.get_editor_world()


TRANS_UE_TO_MUJOCO_SCALE = 1.0  # keep cm
ROT_UE_TO_MUJOCO_SCALE = 1.0  # keep degrees


def export_meshes(actor, export_dir):
  unreal.EditorLevelLibrary.set_selected_level_actors([actor, ])
  name = actor.get_actor_label()
  filename = osp.join(export_dir, f'{name}.gltf')
  task = unreal.AssetExportTask()
  task.set_editor_property('selected', True)
  task.set_editor_property('automated', True)
  task.set_editor_property('prompt', False)
  task.set_editor_property('object', world)
  task.set_editor_property('filename', filename)
  # unreal.GLTFExporter.set_editor_property('export_task', task)
  options = unreal.GLTFExportOptions()
  options.set_editor_property('export_cameras', False)
  options.set_editor_property('export_lights', False)
  options.set_editor_property('export_uniform_scale', TRANS_UE_TO_MUJOCO_SCALE)
  options.set_editor_property('export_unlit_materials', False)
  options.set_editor_property('export_vertex_colors', False)
  selected_actors = unreal.Set(unreal.Actor)
  selected_actors.add(actor)
  print(f'Writing actor {name} to {filename}...')
  count = 0
  N = 10
  while not unreal.GLTFExporter.export_to_gltf(None, filename, options, selected_actors):
  # while not unreal.Exporter.run_asset_export_task(task):
    print(f'Retry {count+1} / {N}')
    time.sleep(0.5)
    if count == 10:
      break
  else:
    return True
  return False


PhysicsConstraint = namedtuple(
  'PhysicsConstraint',
  ['parent', 'child', 'name', 'range', 'ref', 'axis', 'type']
)
MeshComponent = namedtuple('MeshComponent', ['decompose_method', 'decompose_group'])


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--scene_path', required=True)
  parser.add_argument('--actors', default=None,
                      help='Comma separate list of actor names - all others will be excluded if this option is used')
  args = parser.parse_args()
  export_dir = osp.expanduser(osp.join(args.scene_path, 'ue_export'))
  os.makedirs(export_dir, exist_ok=True)
  include_actors = args.actors.split(',') if args.actors else None
  
  filename = osp.join(export_dir, 'actors_information.json')
  if osp.isfile(filename):
    with open(filename, 'r') as f:
      body_properties = json.load(f)
  else:
    body_properties = {}
  
  # Iterate through the actors
  for actor in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor):
    if isinstance(actor, unreal.CameraActor) or isinstance(actor, unreal.SceneCapture2D) or \
      isinstance(actor, unreal.PointLight):
      continue
    actor_name = actor.get_actor_label()
    if 'HDRIBackdrop' in actor_name:
      continue
    if include_actors and (actor_name not in include_actors):
      continue
    
    this_body_properties = {'geoms': {}, }
    # check if actor has geometry
    has_geometry = False
    i = 0
    for c in actor.get_components_by_class(unreal.StaticMeshComponent):
      static_mesh = c.get_editor_property('static_mesh')
      if static_mesh is None:
        print(f'actor {actor_name} component {c.get_name()} does not have a static mesh')
      elif static_mesh.get_name() != 'SM_Dummy':
        # TODO(samarth) get MeshComponent elements from UE component
        c_name = c.get_name()
        if c_name == 'StaticMeshComponent0':
          c_name = actor_name
        this_body_properties['geoms'][c_name] = MeshComponent('coacd', i)._asdict()
        has_geometry = True
        i += 1

    if has_geometry:
      export_meshes(actor, export_dir)
    else:
      print(f'Actor {actor_name}, does not have geometry')
      continue
    
    if isinstance(actor, unreal.StaticMeshActor):
      this_body_properties['moving'] = actor.get_editor_property('static_mesh_component').get_editor_property('body_instance').get_editor_property('simulate_physics')
    else: # TODO(samarth) get the actual flag for the actuated body
      this_body_properties['moving'] = False
    this_body_properties['root_component_name'] = actor.get_editor_property('root_component').get_name()
    body_properties[actor_name] = this_body_properties
    
    constraint_dicts = []
    for component in actor.get_components_by_class(unreal.ActorComponent):
      if not isinstance(component, unreal.PhysicsConstraintComponent):
        continue
      
      r = component.get_editor_property('relative_rotation')
      rot = unreal.Rotator(r.roll, r.pitch, r.yaw)
      
      parent = str(component.get_editor_property('component_name2').get_editor_property('component_name'))
      if (parent == 'DefaultSceneRoot') or (parent == 'StaticMeshComponent0'):
        parent = actor_name
      child = str(component.get_editor_property('component_name1').get_editor_property('component_name'))
      
      constraint_instance = component.get_editor_property('constraint_instance')
      angular_offset = constraint_instance.get_editor_property('angular_rotation_offset')
      constraint_profile = constraint_instance.get_editor_property('profile_instance')
      
      limit = 0.0
      offset = 0.0
      
      linear_limit = constraint_profile.get_editor_property('linear_limit')
      limit = linear_limit.get_editor_property('limit') * TRANS_UE_TO_MUJOCO_SCALE
      
      # prismatic joints
      n_prismatic_dofs = 0
      pc_args = [parent, child, ]
      if linear_limit.get_editor_property('x_motion') != unreal.LinearConstraintMotion.LCM_LOCKED:
        n_prismatic_dofs += 1
        axis = unreal.Vector.FORWARD
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_x_prismatic',
          [-limit, limit],
          offset,
          [axis.x, axis.y, axis.z],
          'slide'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
      pc_args = [parent, child, ]
      if linear_limit.get_editor_property('y_motion') != unreal.LinearConstraintMotion.LCM_LOCKED:
        n_prismatic_dofs += 1
        axis = unreal.Vector.RIGHT
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_y_prismatic',
          [-limit, limit],
          offset,
          [axis.x, axis.y, axis.z],
          'slide'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
      pc_args = [parent, child, ]
      if linear_limit.get_editor_property('z_motion') != unreal.LinearConstraintMotion.LCM_LOCKED:
        n_prismatic_dofs += 1
        axis = unreal.Vector.UP
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_z_prismatic',
          [-limit, limit],
          offset,
          [axis.x, axis.y, axis.z],
          'slide'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
      if n_prismatic_dofs > 1:
        raise Warning(f'{actor_name} has a multi-axis prismatic constraint')
      
      # revolute joints
      n_revolute_dofs = 0
      cone_limit = constraint_profile.get_editor_property('cone_limit')
      pc_args = [parent, child, ]
      if (cone_limit.get_editor_property('swing1_motion') != unreal.AngularConstraintMotion.ACM_LOCKED):
        n_revolute_dofs += 1
        if n_prismatic_dofs > 1:
          raise Warning(f'{actor_name} already has {n_prismatic_dofs} prismatic joints')
        limit = cone_limit.get_editor_property('swing1_limit_degrees') * ROT_UE_TO_MUJOCO_SCALE
        offset = angular_offset.yaw * ROT_UE_TO_MUJOCO_SCALE
        axis = unreal.Vector.UP
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_z_revolute',
          [-limit, limit],
          offset,
          [axis.x, axis.y, axis.z],
          'hinge'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
      pc_args = [parent, child, ]
      if (cone_limit.get_editor_property('swing2_motion') != unreal.AngularConstraintMotion.ACM_LOCKED):
        n_revolute_dofs += 1
        if n_prismatic_dofs > 1:
          raise ValueError(f'{actor_name} already has {n_prismatic_dofs} prismatic joints')
        limit = cone_limit.get_editor_property('swing2_limit_degrees') * ROT_UE_TO_MUJOCO_SCALE
        offset = angular_offset.pitch * ROT_UE_TO_MUJOCO_SCALE
        axis = unreal.Vector.RIGHT
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_y_revolute',
          [-limit, limit],
          offset,
          [axis.x, axis.y, axis.z],
          'hinge'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
      twist_limit = constraint_profile.get_editor_property('twist_limit')
      pc_args = [parent, child, ]
      if twist_limit.get_editor_property('twist_motion') != unreal.AngularConstraintMotion.ACM_LOCKED:
        n_revolute_dofs += 1
        if n_prismatic_dofs > 1:
          raise ValueError(f'{actor_name} already has {n_prismatic_dofs} prismatic joints')
        limit = twist_limit.get_editor_property('twist_limit_degrees') * ROT_UE_TO_MUJOCO_SCALE
        offset = angular_offset.roll * ROT_UE_TO_MUJOCO_SCALE
        # UE applied X rotations are right handed
        # https://github.com/ethz-asl/unreal_airsim/blob/master/docs/coordinate_systems.md
        axis = unreal.Vector.BACKWARD
        axis = axis.rotate(rot).normal()
        pc_args.extend([
          f'{component.get_name()}_x_revolute',
          [-limit, limit],
          offset,
          [axis.x, axis.y, axis.z],
          'hinge'
        ])
        pc = PhysicsConstraint(*pc_args)
        constraint_dicts.append(pc._asdict())
    
    if len(constraint_dicts) > 0:
      filename = osp.join(export_dir, f'{actor_name}_joints.json')
      with open(filename, 'w') as f:
        json.dump(constraint_dicts, f, indent=4)
      print(f'{filename} written')
    
  filename = osp.join(export_dir, 'actors_information.json')
  with open(filename, 'w') as f:
      json.dump(body_properties, f, indent=4)
  print(f'{filename} written')