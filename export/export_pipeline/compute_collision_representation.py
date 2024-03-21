"""
This script decomposes all static meshes into sets of convex regions.
Decomposition method and parameters are based on the parameters chosen in UE, which are captured in 
gltf_scene/actors_information.json.
"""
import argparse
from collections import deque, defaultdict, namedtuple
import copy
import json
import multiprocessing as mp
if mp.current_process().name == 'MainProcess':
  import bpy
import coacd  # needs to be imported after bpy, otherwise terminate called after throwing an instance of 'std::bad_cast'
import numpy as np
import os
import shutil
import subprocess
import trimesh
import utils
import yaml


osp = os.path


StaticMeshComponent = namedtuple('StaticMeshComponent', ['component', 'pos', 'quat'])
MergedStaticMeshComponents = namedtuple('MergedStaticMeshComponents', ['decompose_method', 'components'],
                                        defaults=['', []])


def decimation_lookup(poly_count):
    # Get the adaptive decimation ratio
    if poly_count < 10000:
        return 1.0
    elif poly_count < 50000:
        return 0.8
    elif poly_count < 100000:
        return 0.6
    elif poly_count < 500000:
        return 0.5
    elif poly_count < 1000000:
        return 0.2
    elif poly_count < 5000000:
        return 0.1
    elif poly_count < 10000000:
        return 0.01
    elif poly_count < 20000000:
        return 0.009
    elif poly_count < 30000000:
        return 0.008
    elif poly_count < 40000000:
        return 0.009
    elif poly_count < 50000000:
        return 0.005
    else:
        return 0.001


def get_relative_pose(component):
    """
    gets pose relative to parent
    """
    component.select_set(True)
    pos = copy.deepcopy(component.location)
    xyz = [pos.x, pos.y, pos.z]
    quat = copy.deepcopy(component.rotation_quaternion)
    quat = [quat.w, quat.x, quat.y, quat.z]
    component.select_set(False)
    return pos, quat


def get_relative_pose_and_center(component, scale_factor=np.ones(3)):
    """
    gets pose relative to parent, and centers obj w.r.t. parent
    """
    pos, quat = get_relative_pose(component)
    pos = np.asarray(pos) * scale_factor
    component.select_set(True)
    component.location = (0.0, 0.0, 0.0)
    component.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
    component.select_set(False)
    return pos.tolist(), quat


def process_mesh(args: tuple) -> bool:
    """
    Decompose a mesh into convex regions according to hyperparameters.
    Save the convex regions as well as a convex hull of the mesh in a directory structure.
    args: a tuple containing (path to mesh, hyperparameters, rerun flag).
    Returns a bool indicating success of the operation.
    """
    mesh_path, params, rerun = args
    mesh_dir, mesh_filename = osp.split(mesh_path)
    vhacd_args = params['collision_representation']['vhacd']
    coacd_args = params['collision_representation']['coacd']
    acd_args   = params['collision_representation']['acd']

    decomposed = False
    decompose_dir = osp.join(mesh_dir, acd_args['decompose_method'])
    if rerun and osp.isdir(decompose_dir):
        decomposed = True
        os.remove(mesh_path)
    else:
        os.makedirs(decompose_dir, exist_ok=True)
        mesh_path = shutil.move(mesh_path, decompose_dir)
        mesh_name = os.path.splitext(mesh_filename)[0]
        
        # load mesh and check that it is valid
        mesh: trimesh.Trimesh = trimesh.load(mesh_path, force='mesh')
        if mesh.is_empty:
            volume = 0.0
        else:
            try:
                volume = mesh.bounding_box_oriented.volume
            except ValueError:
                volume = 0.0
        if volume < 1e-6:
            acd_args['decompose_method'] = 'skip'
            print(f'{mesh_name} is too small, volume = {volume:.5f}, will skip')
        elif volume < acd_args['min_volume']:
            acd_args['decompose_method'] = 'none'
            print(f'{mesh_name} is small, volume = {volume:.5f}, will not decompose')

        # save convex hull
        hull_mesh: trimesh.Trimesh = trimesh.convex.convex_hull(mesh)
        hull_filename = osp.join(mesh_dir, 'convex_hull.stl')
        hull_mesh.export(hull_filename)
        print(f'Convex hull {hull_filename} saved.')
        
        if acd_args['decompose_method'] == 'coacd':
            # Call CoACD.
            if coacd_args['auto_threshold']:
                model = coacd_args['threshold_model']
                threshold = max(0.02, volume*model[0] + model[1])
            else:
                threshold = coacd_args['threshold']
            mesh = coacd.Mesh(mesh.vertices, mesh.faces)
            result = coacd.run_coacd(
                mesh,
                threshold=threshold,
                max_convex_hull=coacd_args['max_convex_hull'],
                preprocess_mode=coacd_args['preprocess_mode'],
                preprocess_resolution=coacd_args['preprocess_resolution'],
                resolution=coacd_args['resolution'],
                mcts_nodes=coacd_args['mcts_nodes'],
                mcts_iterations=coacd_args['mcts_iterations'],
                mcts_max_depth=coacd_args['mcts_max_depth'],
                pca=coacd_args['pca'],
                merge=coacd_args['merge'],
                seed=coacd_args['seed'],
                )

            mesh_parts = []
            for vs, fs in result:
                # remove faces with duplicate vertices
                fs = [f for f in fs if len(set(f)) == 3]
                # remove duplicate faces
                fs_set = set()
                final_fs = []
                for f in fs:
                    ff = frozenset(f)  # frozenset() also sorts
                    if ff in fs_set:
                        fs_set.remove(ff)
                    else:
                        fs_set.add(ff)
                        final_fs.append(f)
                mesh = trimesh.Trimesh(vs, final_fs)
                if mesh.is_volume:
                    mesh_parts.append(mesh)

            object_assembled = trimesh.Scene()
            np.random.seed(0)
            for i,p in enumerate(mesh_parts):
                p.visual.vertex_colors[:, :3] = (np.random.rand(3) * 255).astype(np.uint8)
                object_part = trimesh.Scene()
                object_part.add_geometry(p)
                filename = osp.join(decompose_dir, f"{i:03d}{params['common']['CONVEX_DECOMPOSITION_EXT']}")
                object_part.export(filename)
                object_assembled.add_geometry(p)
            if not object_assembled.is_empty:
                object_assembled.export(osp.join(decompose_dir, 'assembled.obj'))
                decomposed = True
        elif acd_args['decompose_method'] == 'vhacd':
            if not vhacd_args['enable']:
                return False

            if shutil.which('TestVHACD') is None:
                print(
                    "V-HACD was enabled but not found in the system path. Either install it "
                    "manually or run `bash install_vhacd.sh`. Skipping decomposition"
                )
                return False

            # Call V-HACD, suppressing output.
            ret = subprocess.run(
                [
                    shutil.which('TestVHACD'),
                    osp.join('convex_decomposition', mesh_filename),
                    "-i",
                    "stl",
                    "-o",
                    params['common']['CONVEX_DECOMPOSITION_EXT'][1:],
                    "-h",
                    f"{vhacd_args['max_output_convex_hulls']}",
                    "-r",
                    f"{vhacd_args['voxel_resolution']}",
                    "-e",
                    f"{vhacd_args['volume_error_percent']}",
                    "-d",
                    f"{vhacd_args['max_recursion_depth']}",
                    "-s",
                    f"{int(not vhacd_args['disable_shrink_wrap'])}",
                    "-f",
                    vhacd_args['fill_mode'],
                    "-v",
                    f"{vhacd_args['max_hull_vert_count']}",
                    "-a",
                    f"{int(not vhacd_args['disable_asyn'])}",
                    "-l",
                    f"{vhacd_args['min_edge_length']}",
                    "-p",
                    f"{int(vhacd_args['split_hull'])}",
                ],
                check=True,
                cwd=mesh_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            output_str = ret.stdout.decode()
            print(output_str)
                    
            if (ret.returncode != 0) or ('VolumeError' in output_str):
                print(f"V-HACD failed on {mesh_path}")
            else: 
                decomposed = True
            shutil.move(osp.join(mesh_dir, vhacd_args['outputs'][1]), osp.join(decompose_dir, 'assembled.stl'))
            os.remove(osp.join(mesh_dir, vhacd_args['outputs'][0]))
            os.remove(osp.join(mesh_dir, vhacd_args['outputs'][2]))
            for src_filename in next(os.walk(decompose_dir))[-1]:
                dst_filename = src_filename.replace(mesh_name, '')
                if dst_filename == '.stl':
                    continue
                shutil.move(osp.join(decompose_dir, src_filename), osp.join(decompose_dir, dst_filename))
        elif acd_args['decompose_method'] == 'none':
            out_filename = osp.join(decompose_dir, 'assembled.stl')
            mesh.export(out_filename)
            out_filename = osp.join(decompose_dir, '000.stl')
            if params['common']['CONVEX_DECOMPOSITION_EXT'] in mesh_path:
                shutil.copy(mesh_path, out_filename)
            else:
                mesh.export(out_filename)
            decomposed = True
        elif acd_args['decompose_method'] == 'skip':
            decomposed = False
            shutil.rmtree(decompose_dir)
        else:
            raise ValueError(f'{mesh_dir} decompose method {acd_args["decompose_method"]} is invalid')
        
        print(f"Removing {mesh_path}")
        os.remove(mesh_path)
        
    if not decomposed:  # delete whole directory
        shutil.rmtree(mesh_dir)
    return decomposed


class CollisionRepresentationComputer(object):
    def __init__(self, scene_path: str, n_workers: int, rerun: bool, include_actors: tuple):
        self.n_workers = min(max(1, n_workers), mp.cpu_count()-1)
        self.rerun = rerun
        self.scene_path = osp.normpath(scene_path)
        with open(osp.join('export_pipeline', 'params.yaml'), 'r') as f:
            self.p = yaml.safe_load(f)
        self.input_dir = osp.join(scene_path, self.p['common']['GLTF_SCENE_DIR_NAME'])
        self.output_dir = osp.join(scene_path, self.p['common']['COLLISION_DIR_NAME'])
        self.input_gltf_filenames = []
        for filename in next(os.walk(self.input_dir))[2]:
            if not filename.endswith('.gltf'):
                continue
            if include_actors and not any([osp.splitext(filename)[0] == i for i in include_actors]):
                continue
            self.input_gltf_filenames.append(osp.join(self.input_dir, filename))


    def _compute_component_descs(self, actor, actor_info: dict, joints_info: dict):
        actor_name = actor.name.split('.')[0]
        KT_node_names = [actor_name, ]
        KT_node_names.extend([j['child'] for j in joints_info.values()])

        # BFS tree traversal and collecting component information
        all_merged_smcs = {}  # maps decompose_dir to MergedStaticMeshComponents 
        components_info = {'static': {}, 'joints': {}}
        # (component, component_parent_dir, component_scale_factor, node_T_parent)
        q = deque([(actor, self.output_dir, np.ones(3), np.eye(4))])
        while len(q):
            component, component_parent_dir, component_scale_factor, node_T_parent = q.popleft()
            parent_name = osp.split(component_parent_dir)[1]
            name = component.name.split('.')[0]
            pos, quat = get_relative_pose_and_center(component, component_scale_factor)
            node_T_component = node_T_parent @ utils.xyzquat_to_T(pos, quat)
            pos, quat = utils.T_to_xyzquat(node_T_component)

            if name in KT_node_names:
                component_parent_dir = osp.join(component_parent_dir, name)
                parent_name = name
                os.makedirs(component_parent_dir, exist_ok=True)
                components_info['static'][name] = {'pos': pos, 'quat': quat, 'meshes': {}}
                node_T_o = np.eye(4)
                pos = [0.0, 0.0, 0.0]
                quat = [1.0, 0.0, 0.0, 0.0]

            is_geom = (component.type == 'MESH') and (component.data.name != 'SM_Dummy')
            is_joint = (component.type == 'EMPTY') and (len(component.children) == 0)

            if is_geom:
                merge_id = actor_info['static_meshes'][name]['merge_id']
                decompose_dir = osp.join(component_parent_dir, self.p['common']['CONVEX_DECOMPOSITION_DIR'],
                                            merge_id)
                decompose_method = actor_info['static_meshes'][name]['decompose_method']
                if decompose_dir not in all_merged_smcs:
                    # New merge_id encountered, so save the transform of its first component w.r.t. nearest 
                    # kinematic tree node. This is necessary, because Convex decomposition will be done in the
                    # object space of this first component and all other components in this merge_id will be merged
                    # into this first component.
                    components_info['static'][parent_name]['meshes'][merge_id] = {
                        'pos': pos,
                        'quat': quat,
                        'decompose_method': decompose_method,
                    }
                    all_merged_smcs[decompose_dir] = \
                        MergedStaticMeshComponents(decompose_method, [StaticMeshComponent(component, pos, quat), ])
                else:
                    if all_merged_smcs[decompose_dir].decompose_method != decompose_method:
                        raise ValueError(f'{decompose_dir} is marked to be decomposed by {decompose_method},'
                                        'which does not match with other meshes to be merged with it.')
                    all_merged_smcs[decompose_dir].components.append(StaticMeshComponent(component, pos, quat))

            if is_joint:
                joint_name = utils.name_in_names(name, joints_info.keys())
                components_info['joints'][joint_name] = {'pos':  pos, 'quat': quat}

            for child_component in component.children:
                q.append((child_component, component_parent_dir, component_scale_factor*component.scale, node_T_o))
        return components_info, all_merged_smcs


    def _export_component_descs(self, actor_name, components_info: dict, all_merged_smcs: dict):
        actor_dir = osp.join(self.output_dir, actor_name)
        filename = osp.join(actor_dir, 'components.json')
        with open(filename, 'w') as f:
            json.dump(components_info, f, indent=4)


    def _merge_decimate_export_smcs(self, all_merged_smcs: dict):
        process_mesh_args = []
        for item in all_merged_smcs.items():
            decompose_dir: str = item[0]
            merged_smcs: MergedStaticMeshComponents = item[1]
            
            # transform all components into the nearest KT node's coordinate frame
            smc: StaticMeshComponent
            for smc in merged_smcs.components:
                smc.component.location = smc.pos
                smc.component.rotation_quaternion = smc.quat
                smc.component.select_set(True)

            # merge them into the first component
            first_component = merged_smcs.components[0].component
            bpy.context.view_layer.objects.active = first_component
            bpy.ops.object.join()

            # merged geometry convex decomposition will happen in the coordinate system of the first component
            first_component.location = (0.0, 0.0, 0.0)
            first_component.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)

            # decimate merged geometry
            decimate_modifier = first_component.modifiers.new(name='Decimate', type='DECIMATE')
            decimate_modifier.ratio = decimation_lookup(len(first_component.data.polygons))

            # export merged geometry in STL format, for convex decomposition libraries
            os.makedirs(decompose_dir, exist_ok=True)
            stl_filepath = osp.join(decompose_dir, "geom.stl")
            if (not self.rerun) or (not osp.isfile(stl_filepath)):
                bpy.ops.export_mesh.stl(
                    filepath=stl_filepath,
                    check_existing=False,
                    use_selection=True,
                    use_mesh_modifiers=True,
                    batch_mode='OFF',
                    global_scale=1.0,
                    axis_forward='Y',
                    axis_up='Z',
                    ascii=False,
                )
                print(f'{stl_filepath} written')
            bpy.ops.object.select_all(action='DESELECT')

            params = copy.deepcopy(self.p)
            params['collision_representation']['acd']['decompose_method'] = merged_smcs.decompose_method
            process_mesh_args.append((stl_filepath, params, self.rerun))
        return process_mesh_args


    def run(self):
        if not self.rerun:
            shutil.rmtree(self.output_dir, ignore_errors=True)

        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()
        
        with open(osp.join(self.input_dir, 'actors_information.json'), 'r') as f:
            actors_information = json.load(f)
        
        process_mesh_args = []
        for input_gltf_filename in self.input_gltf_filenames:
            # import actor GLTF
            bpy.ops.import_scene.gltf(filepath=input_gltf_filename)
            actor = bpy.context.object  # recently imported object
            actor_name = actor.name.split('.')[0]
            
            # load joints info for this actor
            joint_filename = input_gltf_filename.replace('.gltf', '_joints.json')
            try:
                with open(joint_filename, 'r') as f:
                    joints_info = json.load(f)
            except FileNotFoundError:
                joints_info = {}
            joints_info = {j['name']: j for j in joints_info}

            # reason by kinematic tree to decide which StaticMeshComponents (SMCs) will be merged, collect info for
            # components
            components_info, all_merged_smcs = self._compute_component_descs(actor, actors_information[actor_name],
                                                                             joints_info)
            
            # save the component infos
            self._export_component_descs(actor_name, components_info, all_merged_smcs)

            # merge SMCs, decimate the merged geometry, export it for convex decomposition
            process_mesh_args.extend(self._merge_decimate_export_smcs(all_merged_smcs))
            
            # delete actor GLTF
            bpy.ops.object.select_all(action='SELECT')
            bpy.ops.object.delete()
        
        # do the convex decomposition
        print("Convex decomposition...")
        # This for loop is needed to ensure memory is freed after each batch if size self.n_workers
        # If the entire process_mesh_args is given to a mp.Pool(self.n_workers), the parallel workers are re-used when
        # len(process_mesh_args) > self.n_workers, and their memory is not freed after each task. This consumes a
        # vast amount of memory. mp.Pool(self.n_workers, maxtasksperchild=1) does not seem to work. 
        for start_idx in range(0, len(process_mesh_args), self.n_workers):
            with mp.Pool(self.n_workers) as p:
                p.map(process_mesh, process_mesh_args[start_idx : start_idx+self.n_workers])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--pipeline_dir', required=True)
    parser.add_argument('--rerun', action='store_true')
    parser.add_argument('--actors', default=None)
    parser.add_argument('--scene_id', default=None)
    parser.add_argument('--num_parallel_workers', type=int, default=mp.cpu_count()-1)
    args = parser.parse_args()
    
    include_actors = tuple(args.actors.split(',')) if args.actors else ()
    
    for scene_name in sorted(next(os.walk(args.pipeline_dir))[1]):
        if (args.scene_id is not None) and (scene_name != args.scene_id):
            continue
        print(f'############# Scene {scene_name} ############')
        crc = CollisionRepresentationComputer(osp.join(args.pipeline_dir, scene_name), args.num_parallel_workers, args.rerun, include_actors)
        crc.run()