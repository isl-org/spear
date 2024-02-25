#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import mayavi.mlab
import numpy as np
import os
import pathlib
import posixpath
import trimesh
import scipy.spatial.transform
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
parser.add_argument("--scene_id", required=True)
parser.add_argument("--visual_parity_with_unreal", action="store_true")
parser.add_argument("--ignore_actors")
args = parser.parse_args()

if args.ignore_actors is not None:
    ignore_actors = args.ignore_actors.split(",")
else:
    ignore_actors = []

scene_component_classes = ["SceneComponent", "StaticMeshComponent"]
static_mesh_component_classes = ["StaticMeshComponent"]

origin_scale_factor = 1.0
mesh_opacity = 1.0

c_x_axis = (1.0, 0.0, 0.0)
c_y_axis = (0.0, 1.0, 0.0)
c_z_axis = (0.0, 0.0, 1.0)
c_face   = (0.75,0.75,0.75)

origin_world = np.array([[0.0,   0.0,   0.0]])
x_axis_world = np.array([[100.0, 0.0,   0.0]])
y_axis_world = np.array([[0.0,   100.0, 0.0]])
z_axis_world = np.array([[0.0,   0.0,   100.0]])

# Swap y and z coordinates to match the visual appearance of the Unreal editor.
if args.visual_parity_with_unreal:
    # x_axis_world, y_axis_world, z_axis_world = x_axis_world, z_axis_world, y_axis_world
    origin_world = origin_world[:,[0,2,1]]
    x_axis_world = x_axis_world[:,[0,2,1]]
    y_axis_world = y_axis_world[:,[0,2,1]]
    z_axis_world = z_axis_world[:,[0,2,1]]


def process_scene():
    json_file = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_scene_json", "unreal_scene.json"))

    with open(json_file, "r") as f:
        unreal_scene_json = json.load(f)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         x_axis_world[:,0], x_axis_world[:,1], x_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_x_axis)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         y_axis_world[:,0], y_axis_world[:,1], y_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_y_axis)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         z_axis_world[:,0], z_axis_world[:,1], z_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_z_axis)

    # It is possible for an actor not to have a root component.
    for actor_name, actor_desc in list(unreal_scene_json.items()):
        if actor_name not in args.ignore_actors and "root_component" in actor_desc.keys():
            spear.log("Processing actor: ", actor_name)
            root_component_desc = actor_desc["root_component"]
            component_desc = list(root_component_desc.values())[0]

            draw_components(
                component_desc,
                world_from_component_transform_func=world_from_component_transform_using_relative_lrs,
                world_from_component_transform_data={
                    "location": np.zeros(3),
                    "rotation": scipy.spatial.transform.Rotation.from_euler("xyz", [0.0, 0.0, 0.0]),
                    "scale":    np.ones(3)
                },
                log_prefix_str="    ")

    mayavi.mlab.show()


def draw_components(component_desc, world_from_component_transform_func, world_from_component_transform_data, log_prefix_str=""):

    component_class = component_desc["class"]
    if component_class in scene_component_classes:

        spear.log(log_prefix_str, "Processing SceneComponent: ", component_desc["name"])
        M_world_from_component, world_from_component_transform_data = \
            world_from_component_transform_func(component_desc, world_from_component_transform_data)

        if component_class in static_mesh_component_classes:
            spear.log(log_prefix_str, "    Component is a StaticMeshComponent.")
            static_mesh_desc = component_desc["editor_properties"]["static_mesh"]

            if static_mesh_desc is not None:
                spear.log(log_prefix_str, "    Component has a valid StaticMesh.")
                static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_desc["path"])
    
                if static_mesh_asset_path.parts[:4] == ("/", "Game", "Scenes", args.scene_id):

                    obj_path_suffix = posixpath.join(*static_mesh_asset_path.parts[4:]) + ".obj"
                    numerical_parity_obj_path = \
                        os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_geometry", "numerical_parity", obj_path_suffix))

                    spear.log(log_prefix_str, "    StaticMesh asset path: ", static_mesh_asset_path)
                    spear.log(log_prefix_str, "    OBJ file:              ", numerical_parity_obj_path)

                    mesh = trimesh.load_mesh(numerical_parity_obj_path, process=False, validate=False)

                    V_component = np.matrix(np.c_[mesh.vertices, np.ones(mesh.vertices.shape[0])]).T
                    V_world = M_world_from_component*V_component
                    assert np.allclose(V_world[3,:], 1.0)

                    mesh.vertices = V_world.T.A[:,0:3]

                    # Swap y and z coordinates to match the visual appearance of the Unreal editor.
                    if args.visual_parity_with_unreal:
                        mesh.vertices = mesh.vertices[:,[0,2,1]]

                    mayavi.mlab.triangular_mesh(
                        mesh.vertices[:,0], mesh.vertices[:,1], mesh.vertices[:,2], mesh.faces, representation="surface", color=c_face, opacity=mesh_opacity)

        for child_component_desc in component_desc["children_components"].values():
            draw_components(
                child_component_desc, world_from_component_transform_func, world_from_component_transform_data, log_prefix_str=log_prefix_str + "    ")


def world_from_component_transform_using_relative_lrs(component_desc, world_from_component_transform_data):

    absolute_location = component_desc["editor_properties"]["absolute_location"]
    absolute_rotation = component_desc["editor_properties"]["absolute_rotation"]
    absolute_scale = component_desc["editor_properties"]["absolute_scale"]
    relative_location_x = component_desc["editor_properties"]["relative_location"]["editor_properties"]["x"]
    relative_location_y = component_desc["editor_properties"]["relative_location"]["editor_properties"]["y"]
    relative_location_z = component_desc["editor_properties"]["relative_location"]["editor_properties"]["z"]
    relative_rotation_roll = component_desc["editor_properties"]["relative_rotation"]["editor_properties"]["roll"]
    relative_rotation_pitch = component_desc["editor_properties"]["relative_rotation"]["editor_properties"]["pitch"]
    relative_rotation_yaw = component_desc["editor_properties"]["relative_rotation"]["editor_properties"]["yaw"]
    relative_scale3d_x = component_desc["editor_properties"]["relative_scale3d"]["editor_properties"]["x"]
    relative_scale3d_y = component_desc["editor_properties"]["relative_scale3d"]["editor_properties"]["y"]
    relative_scale3d_z = component_desc["editor_properties"]["relative_scale3d"]["editor_properties"]["z"]

    roll  = np.deg2rad(-relative_rotation_roll)
    pitch = np.deg2rad(-relative_rotation_pitch)
    yaw   = np.deg2rad(relative_rotation_yaw)

    l_parent_from_component = np.array([relative_location_x, relative_location_y, relative_location_z])
    R_parent_from_component = scipy.spatial.transform.Rotation.from_euler("xyz", [roll, pitch, yaw])
    s_parent_from_component = np.array([relative_scale3d_x, relative_scale3d_y, relative_scale3d_z])

    eps = 0.000001
    assert np.all(s_parent_from_component > eps)

    #
    # See Engine\Source\Runtime\Engine\Private\Components\SceneComponent.cpp for details:
    #
    #     FORCEINLINE FTransform CalcNewComponentToWorld(const FTransform& NewRelativeTransform, const USceneComponent* Parent = nullptr, FName SocketName = NAME_None) const
    #     {
    #         // ...
    #         return NewRelativeTransform * Parent->GetSocketTransform(SocketName);
    #         // ...
    #     }
    #
    # See Engine\Source\Runtime\Core\Public\Math\TransformNonVectorized.h for details:
    #
    #     template<typename T>
    #     FORCEINLINE TTransform<T> TTransform<T>::operator*(const TTransform<T>& Other) const
    #     {
    #         // ...
    #         Multiply(&Output, this, &Other);
    #         // ...    
    #     }
    #
    #     template<typename T>
    #     FORCEINLINE void TTransform<T>::Multiply(TTransform<T>* OutTransform, const TTransform<T>* A, const TTransform<T>* B)
    #     {
    #         // ...
    #         OutTransform->Rotation = B->Rotation*A->Rotation;
    #         OutTransform->Scale3D = A->Scale3D*B->Scale3D;
    #         OutTransform->Translation = B->Rotation*(B->Scale3D*A->Translation) + B->Translation;
    #         // ...
    #     }
    #

    l_world_from_parent = world_from_component_transform_data["location"]
    R_world_from_parent = world_from_component_transform_data["rotation"]
    s_world_from_parent = world_from_component_transform_data["scale"]

    l_world_from_component = R_world_from_parent.apply(s_world_from_parent*l_parent_from_component) + l_world_from_parent
    R_world_from_component = R_world_from_parent*R_parent_from_component
    s_world_from_component = s_parent_from_component*s_world_from_parent

    if absolute_location:
        l_world_from_component = l_parent_from_component

    if absolute_rotation:
        R_world_from_component = R_parent_from_component

    if absolute_scale:
        s_world_from_component = s_parent_from_component

    M_l_world_from_component = np.block([[np.identity(3),                     np.matrix(l_world_from_component).T], [np.zeros(3), 1.0]])
    M_R_world_from_component = np.block([[R_world_from_component.as_matrix(), np.matrix(np.zeros(3)).T],            [np.zeros(3), 1.0]])
    M_s_world_from_component = np.block([[np.diag(s_world_from_component),    np.matrix(np.zeros(3)).T],            [np.zeros(3), 1.0]])

    M_world_from_component = M_l_world_from_component*M_R_world_from_component*M_s_world_from_component

    # Check the M_world_from_component matrix that we compute here against the native unreal.SceneComponent.get_world_transform()
    # function. We store the result of this function in our exported JSON file, so we simply compare our computed matrix against
    # the stored result. Note that each "plane" below refers to a column, so in this sense, Unreal editor properies store matrices
    # in column-major order.
    M_world_from_component_00_ = component_desc["world_transform_matrix"]["editor_properties"]["x_plane"]["editor_properties"]["x"]
    M_world_from_component_10_ = component_desc["world_transform_matrix"]["editor_properties"]["x_plane"]["editor_properties"]["y"]
    M_world_from_component_20_ = component_desc["world_transform_matrix"]["editor_properties"]["x_plane"]["editor_properties"]["z"]
    M_world_from_component_30_ = component_desc["world_transform_matrix"]["editor_properties"]["x_plane"]["editor_properties"]["w"]
    M_world_from_component_01_ = component_desc["world_transform_matrix"]["editor_properties"]["y_plane"]["editor_properties"]["x"]
    M_world_from_component_11_ = component_desc["world_transform_matrix"]["editor_properties"]["y_plane"]["editor_properties"]["y"]
    M_world_from_component_21_ = component_desc["world_transform_matrix"]["editor_properties"]["y_plane"]["editor_properties"]["z"]
    M_world_from_component_31_ = component_desc["world_transform_matrix"]["editor_properties"]["y_plane"]["editor_properties"]["w"]
    M_world_from_component_02_ = component_desc["world_transform_matrix"]["editor_properties"]["z_plane"]["editor_properties"]["x"]
    M_world_from_component_12_ = component_desc["world_transform_matrix"]["editor_properties"]["z_plane"]["editor_properties"]["y"]
    M_world_from_component_22_ = component_desc["world_transform_matrix"]["editor_properties"]["z_plane"]["editor_properties"]["z"]
    M_world_from_component_32_ = component_desc["world_transform_matrix"]["editor_properties"]["z_plane"]["editor_properties"]["w"]
    M_world_from_component_03_ = component_desc["world_transform_matrix"]["editor_properties"]["w_plane"]["editor_properties"]["x"]
    M_world_from_component_13_ = component_desc["world_transform_matrix"]["editor_properties"]["w_plane"]["editor_properties"]["y"]
    M_world_from_component_23_ = component_desc["world_transform_matrix"]["editor_properties"]["w_plane"]["editor_properties"]["z"]
    M_world_from_component_33_ = component_desc["world_transform_matrix"]["editor_properties"]["w_plane"]["editor_properties"]["w"]

    M_world_from_component_ = np.matrix([
        [M_world_from_component_00_, M_world_from_component_01_, M_world_from_component_02_, M_world_from_component_03_],
        [M_world_from_component_10_, M_world_from_component_11_, M_world_from_component_12_, M_world_from_component_13_],
        [M_world_from_component_20_, M_world_from_component_21_, M_world_from_component_22_, M_world_from_component_23_],
        [M_world_from_component_30_, M_world_from_component_31_, M_world_from_component_32_, M_world_from_component_33_]])
    assert np.allclose(M_world_from_component, M_world_from_component_)

    return M_world_from_component, {"location": l_world_from_component, "rotation": R_world_from_component, "scale": s_world_from_component}


if __name__ == '__main__':
    process_scene()
