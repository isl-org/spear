# MuJoCo Export Pipeline
This code generates a MuJoCo replica of a SPEAR scene.

## Usage
All commands to be run from the `export` directory. Assume that the name of the scene is stored in the `SCENE_NAME`
environment variable.
- If you have not done so already, follow the SPEAR [Getting Started](../docs/getting_started.md) instructions, which
will result in the `spear-env` conda environment.
- Build V-HACD using the [readme instructions](v-hacd/README.md).
- Create the scene directory: `mkdir -p scenes/${SCENE_NAME}`.
- Open the scene map in Unreal Editor (UE).
- From the UE Python console, run the script `path/to/spear/export/export_meshes_and_joints_from_ue.py --scene_path ${SCENE_NAME}`.
- Activate the SPEAR Conda environment, and run the processing script.
```bash
(base) $ conda activate spear-env
(spear-env) $ python mujoco_export_pipeline/export_scene.py -n <num parallel workers> --scene_path ${SCENE_NAME}
```

## Assumptions
- The floor geometry must have the floor spread across the X-Y plane, and Z axis pointing upwards.
- Dummy meshes in UE are called `SM_Dummy`
- Within each `Actor`, no two `StaticMeshComponent`s have the same name, and no two `PhysicsConstraintComponent`s have
the same name.

## TODO (maybe in future PRs)
- [ ] Articulated objects are currently assumed to be static w.r.t. world i.e. `"moving": false` in their entry in
`<scene_path>/ue_export/actors_information.json`. This needs to be derived by checking if `simulate_physics` is set for
any component other than the `DefaultSceneRoot` and children of `PhysicsContstraintComponents`.
- [x] Implement numerical parity between UE and MuJoCo representations i.e. in the joint positions, object poses, etc.
Currently UE uses cm and degrees. Exporting to GLTF converts it to m and radians, which is also the unit system used in
MuJoCo. 
- [x] Check if MuJoCo can operate in degrees.
- [x] More accurate gravity value in scene_include.xml.
- [x] Check parity of mesh vertices.
- [x] Check matching behaviour of UE and MuJoCo given same joint targets.

## Miscellaneous
UE Editor REPL code for frequently used debugging operations:

```python
import unreal

ss = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world = ss.get_editor_world()
all_actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
actor = [a for a in all_actors if a.get_actor_label() == 'door_00'][0]
components = [c for c in actor.get_components_by_class(unreal.PhysicsConstraintComponent)]
c = components[0]
profile = c.get_editor_property('constraint_instance').get_editor_property('profile_instance')
```
