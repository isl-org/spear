"""
Run this script from UE
Prints the location and rotation of the given component, relative to its parent
"""
import argparse
import unreal


def print_joint_state(actor_name: str, component_name: str):
  ss = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
  world = ss.get_game_world()
  # world = ss.get_editor_world()
  try:
    actor = next(filter(lambda a: a.get_actor_label() == actor_name, unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)))
  except StopIteration:
    print(f'Scene does not have any unreal.Actor named {actor_name}')
    raise
  try:
    comp = next(filter(lambda c: c.get_name() == component_name, actor.get_components_by_class(unreal.StaticMeshComponent)))
  except StopIteration:
    print(f'unreal.Actor {actor_name} does not have any unreal.PhysicsConstraintComponent named {component_name}')
    raise
  print(comp.get_editor_property('relative_location'))
  print(comp.get_editor_property('relative_rotation'))


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--actor_name', required=True)
  parser.add_argument('--component_name', required=True)
  args = parser.parse_args()
  print_joint_state(args.actor_name, args.component_name)