# MuJoCo Export Pipeline
This code generates a MuJoCo replica of a SPEAR scene.

## Usage
All commands to be run from the `export` directory. Assume that the name of the scene is stored in the `SCENE_NAME`
environment variable.
- If you have not done so already, follow the SPEAR [Getting Started](../docs/getting_started.md) instructions, which
will result in the `spear-env` conda environment.
- Install PyYAML for UE Python: `/path/to/ue/python3 -m pip install -r /path/to/spear/export/ue_requirements.txt`. Here, `/path/to/ue/python3` is the full path to UE's Python interpreter. You can get it by running `import os; print(os.__file__)` in UE's Python REPL console and removing the `os.py` at the end of the print output.
- Build V-HACD using the [readme instructions](v-hacd/README.md).
- Create the scene directory: `mkdir -p scenes/${SCENE_NAME}`.
- Open the scene map in Unreal Editor (UE).
- From the UE Python console, run the script `/path/to/spear/export/export_pipeline/compute_gltf_representation.py --scene_path /path/to/spear/export/scenes/${SCENE_NAME}`.
- Activate the SPEAR Conda environment, and run the processing script.
```bash
(base) $ cd /path/to/spear/export
(base) $ conda activate spear-env
(spear-env) $ python export_pipeline/compute_collision_representation.py --pipeline_dir scenes --scene_id ${SCENE_NAME}
(spear-env) $ python export_pipeline/compute_mujoco_representation.py --pipeline_dir scenes --scene_id ${SCENE_NAME}
(spear-env) $ python -m mujoco.viewer --mjcf scenes/${SCENE_NAME}/mujoco_scene/scene.xml
```
**NOTE**: The `--pipeline_dir` argument must be relative to `/path/to/spear/export`. 

## Assumptions
- The floor geometry must have the floor spread across the X-Y plane, and Z axis pointing upwards.
- Dummy meshes in UE are called `SM_Dummy`
- Within each `Actor`, no two `StaticMeshComponent`s have the same name, and no two `PhysicsConstraintComponent`s have
the same name.
- All joints have only one DoF.
- A Component cannot be the child of two PhysicsConstraintComponents.
- No component is named `convex_decomposition`.

## TODO (maybe in future PRs)
- [ ] Articulated objects are currently assumed to be static w.r.t. world i.e. `"simulate_physics": false` in their
entry in `<scene_path>/gltf_scene/actors_information.json`. This needs to be derived by checking if
`simulate_physics` is set for any component other than the `DefaultSceneRoot` and children of
`PhysicsContstraintComponents`.
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

## Coordinate Systems
Unreal Engine uses a left-handed coordinate system, but its GLTF exporter exports actors (geometry, poses) in a right-handed system.
So [`export_pipeline/compute_gltf_representation.py`](export_pipeline/compute_gltf_representation.py) exports joint
information in a right-handed coordinate system, to maintain consistency with the UE GLTF exporter. This is done through
the choice of axes of rotation for revolute joints. Logic:
```python
def get_rotation_matrix_from_unreal_pyr(unreal_pyr):
    #
    # Unreal's FRotator constructor takes 3 Euler angles specified in degrees and in pitch-yaw-roll order,
    # so we assume that our input array unreal_pyr is also specified in degrees and in pitch-yaw-roll order.
    # However, scipy assumes that Euler angles are given in radians by default, so we need to convert
    # degrees to radians.
    #
    # Unreal defines pitch-yaw-roll angles according to the following conventions, which can be verified by
    # manual inspection in the editor.
    #     A positive pitch is a rotation around Y, starting from +X and rotating towards +Z
    #     A positive yaw   is a rotation around Z, starting from +X and rotating towards +Y
    #     A positive roll  is a rotation around X, starting from +Z and rotating towards +Y
    #
    # On the other hand, scipy defines a rotation around each axis according to the following conventions,
    # which can be verified by manually inspecting the output of
    # scipy.spatial.transform.Rotation.from_euler(...).as_matrix().
    #     A rotation around X by theta radians is defined by the following matrix,
    #         [[1 0 0  ]
    #          [0 c -s ] 
    #          [0 s c  ]], where c=cos(theta) and s=sin(theta)
    #     A rotation around Y by theta radians is defined by the following matrix,
    #         [[c  0 s ]
    #          [0  1 0 ] 
    #          [-s 0 c ]], where c=cos(theta) and s=sin(theta)
    #     A rotation around Z by theta radians is defined by the following matrix,
    #         [[c -s 0 ]
    #          [s c  0 ] 
    #          [0 0  1 ]], where c=cos(theta) and s=sin(theta)
    #
    # These conventions conflict. We therefore need to negate Unreal's pitch (rotation around Y) and roll
    # (rotation around X) but not yaw (rotation around Z).
    #
    
    pitch = np.deg2rad(-unreal_pyr[0])
    yaw   = np.deg2rad(unreal_pyr[1])
    roll  = np.deg2rad(-unreal_pyr[2])

    #
    # Unreal applies pitch-yaw-roll Euler angles in world-space in the following order, which can be
    # verified by manual inspection the editor.
    #     1. rotate around world-space X by roll degrees
    #     2. rotate around world-space Y by pitch degrees
    #     3. rotate around world-space Z by yaw degrees.
    # 
    # So we define our rotation matrix as follows,
    #     R_x = np.matrix(scipy.spatial.transform.Rotation.from_euler("x", roll).as_matrix())
    #     R_y = np.matrix(scipy.spatial.transform.Rotation.from_euler("y", pitch).as_matrix())
    #     R_z = np.matrix(scipy.spatial.transform.Rotation.from_euler("z", yaw).as_matrix())
    #     R   = R_z*R_y*R_x
    # which is equivalent to the following expression,
    #     R   = np.matrix(scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix())
    #
    
    R = np.matrix(scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix())
```

Later,
[`export_pipeline/compute_mujoco_representation.py`](export_pipeline/compute_mujoco_representation.py) converts it 
back to the left-handed coordinate system (see the usage of `pose_rl_to_lh()`
from [`export_pipeline/utils.py`](utils.py)). So finally, the scene appears flipped around one axis in the MuJoCo
viewer because MuJoCo assumes a right-handed coordinate system.