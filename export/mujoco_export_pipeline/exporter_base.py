from abc import ABC, abstractmethod, abstractclassmethod
from collections import deque, defaultdict
import copy
import json
import multiprocessing as mp
if mp.current_process().name == 'MainProcess':
    import bpy
import coacd  # needs to be imported after bpy, otherwise terminate called after throwing an instance of 'std::bad_cast'
import numpy as np
import os
import params
import shutil
import subprocess
import transforms3d.quaternions as txq
import trimesh
import utils


osp = os.path


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


def get_relative_pose(obj):
    """
    gets pose relative to parent
    """
    obj.select_set(True)
    xyz = copy.deepcopy(obj.location)
    xyz = [xyz.x, xyz.y, xyz.z]
    quat = copy.deepcopy(obj.rotation_quaternion)
    quat = [quat.w, quat.x, quat.y, quat.z]
    obj.select_set(False)
    return xyz, quat


def get_relative_pose_and_center(obj, scale_factor=np.ones(3)):
    """
    gets pose relative to parent, and centers obj w.r.t. parent
    """
    xyz, quat = get_relative_pose(obj)
    xyz = np.asarray(xyz) * scale_factor
    obj.select_set(True)
    obj.location = (0.0, 0.0, 0.0)
    obj.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
    obj.select_set(False)
    return xyz.tolist(), quat        


class ExporterBase(ABC):
    """
    Base class for exporting the SPEAR UE scene to an external physics engine. Broadly, it performs these steps:
    - Decompose each StaticMesh into convex meshes.
    - Compose all meshes belonging to a Component with appropriate offsets.
    - Compose these along with joints, if present, into physics engine structure corresponding to UE Actor e.g. MuJoCo body
    """
    def __init__(self, scene_path: str, n_workers: int, rerun: bool, include_objects: str) -> None:
        """
        scene_path: scene directory, same as the scene_path provided to export_meshes_and_joints_from_ue.py
        n_workers: number of parallel workers
        rerun: if True, will skip StaticMeshes which have been decomposed by a previous run. Else, will delete them and re-process
        include_objects: comma-separated string of Actor names. If not None, will process StaticMeshes in only those Actors
        """
        self.n_workers = min(max(1, n_workers), mp.cpu_count()-1)
        self.rerun = rerun
        ue_export_path = osp.join(scene_path, 'ue_export')
        self.input_gltf_filenames = {}
        for filename in next(os.walk(ue_export_path))[2]:
            if not filename.endswith('.gltf'):
                continue
            if include_objects and not any([(osp.splitext(filename)[0] == i) for i in include_objects.split(',')]):
                continue
            joint_filename = filename.replace('.gltf', '_joints.json')
            self.input_gltf_filenames[osp.join(ue_export_path, filename)] = osp.isfile(osp.join(ue_export_path, joint_filename))
        self.scene_path = osp.normpath(scene_path)
        self.output_dir = None 
        
        # Clear scene
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()
        
        # Import the .gltf scene files
        self.object_has_joint = {}
        for gltf_filename, has_joint in self.input_gltf_filenames.items():
          try:
              print(f'importing {gltf_filename}')
              bpy.ops.import_scene.gltf(filepath=gltf_filename)
              object_name = bpy.context.object.name.split('.')[0]
              self.object_has_joint[object_name] = has_joint
          except RuntimeError:
              print('Failed')

        # whether each object is moving or not
        with open(osp.join(ue_export_path, 'actors_information.json'), 'r') as f:
            self.actors_information = json.load(f)
    
    
    @abstractmethod
    def generate_scene(self):
        pass


    @abstractclassmethod
    def assemble_mesh(cls, args):
        pass


    @abstractclassmethod
    def assemble_object_with_joints(cls, args):
        pass


    @abstractclassmethod
    def check_mesh_validity(cls, filename: str) -> bool:
        return False


    @classmethod
    def decompose_convex(cls, args):
        fullpath, output_folder, scene_path, cvx_dir, acd_args, xyz, quat, decompose_in_bodies, rerun = args
        obj_dir, obj_file = os.path.split(fullpath)

        # Copy the obj file to the temporary directory.
        decomposed = False
        cvx_path = osp.join(obj_dir, cvx_dir)
        if rerun and osp.isdir(cvx_path):
            decomposed = True
        else:
            os.makedirs(cvx_path, exist_ok=True)
            shutil.copy(fullpath, cvx_path)

            obj_filename = osp.join(cvx_path, obj_file)
            obj_name = os.path.splitext(obj_file)[0]
            
            if cls.check_mesh_validity(obj_filename):
                mesh: trimesh.Trimesh = trimesh.load(obj_filename, force='mesh')
                if mesh.is_empty:
                    volume = 0.0
                else:
                    try:
                        volume = mesh.bounding_box_oriented.volume
                    except ValueError:
                        volume = 0.0
                if volume < 1e-6:
                    acd_args.decompose_method = 'skip'
                    print(f'{obj_name} is too small, volume = {volume:.5f}, will skip')
                elif volume < acd_args.min_volume:
                    acd_args.decompose_method = 'none'
                    print(f'{obj_name} is small, volume = {volume:.5f}, will not decompose')
                
                if acd_args.decompose_method == 'coacd':
                    # Call CoACD.
                    if acd_args.coacd_args.auto_threshold:
                        model = acd_args.coacd_args.threshold_model
                        threshold = max(0.02, volume*model[0] + model[1])
                    else:
                        threshold = acd_args.coacd_args.threshold
                    mesh = coacd.Mesh(mesh.vertices, mesh.faces)
                    result = coacd.run_coacd(
                        mesh,
                        threshold=threshold,
                        max_convex_hull=acd_args.coacd_args.max_convex_hull,
                        preprocess_mode=acd_args.coacd_args.preprocess_mode,
                        preprocess_resolution=acd_args.coacd_args.preprocess_resolution,
                        resolution=acd_args.coacd_args.resolution,
                        mcts_nodes=acd_args.coacd_args.mcts_nodes,
                        mcts_iterations=acd_args.coacd_args.mcts_iterations,
                        mcts_max_depth=acd_args.coacd_args.mcts_max_depth,
                        pca=acd_args.coacd_args.pca,
                        merge=acd_args.coacd_args.merge,
                        seed=acd_args.coacd_args.seed,
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
                    i = 0
                    for p in mesh_parts:
                        p.visual.vertex_colors[:, :3] = (np.random.rand(3) * 255).astype(np.uint8)
                        object_part = trimesh.Scene()
                        object_part.add_geometry(p)
                        filename = osp.join(cvx_path, f'{obj_name}{i:03d}.stl')
                        object_part.export(filename)
                        if cls.check_mesh_validity(filename):
                            object_assembled.add_geometry(p)
                            i+=1
                    if not object_assembled.is_empty:
                        object_assembled.export(osp.join(obj_dir, f'{obj_name}_{cvx_dir}.obj'))
                        decomposed = True
                elif acd_args.decompose_method == 'vhacd':
                    if not acd_args.vhacd_args.enable:
                        return False

                    if params.VHACD_EXECUTABLE is None:
                        print(
                            "V-HACD was enabled but not found in the system path. Either install it "
                            "manually or run `bash install_vhacd.sh`. Skipping decomposition"
                        )
                        return False

                    # Call V-HACD, suppressing output.
                    ret = subprocess.run(
                        [
                            f"{params.VHACD_EXECUTABLE}",
                            osp.join(cvx_dir, obj_file),
                            "-i",
                            "stl",
                            "-o",
                            "stl",
                            "-h",
                            f"{acd_args.vhacd_args.max_output_convex_hulls}",
                            "-r",
                            f"{acd_args.vhacd_args.voxel_resolution}",
                            "-e",
                            f"{acd_args.vhacd_args.volume_error_percent}",
                            "-d",
                            f"{acd_args.vhacd_args.max_recursion_depth}",
                            "-s",
                            f"{int(not acd_args.vhacd_args.disable_shrink_wrap)}",
                            "-f",
                            f"{acd_args.vhacd_args.fill_mode.name.lower()}",
                            "-v",
                            f"{acd_args.vhacd_args.max_hull_vert_count}",
                            "-a",
                            f"{int(not acd_args.vhacd_args.disable_async)}",
                            "-l",
                            f"{acd_args.vhacd_args.min_edge_length}",
                            "-p",
                            f"{int(acd_args.vhacd_args.split_hull)}",
                        ],
                        check=True,
                        cwd=obj_dir,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                    )
                    output_str = ret.stdout.decode()
                    print(output_str)
                            
                    if (ret.returncode != 0) or ('VolumeError' in output_str):
                        print(f"V-HACD failed on {fullpath}")
                    else: 
                        decomposed = True
                    # VHACD not being able to generate a convex hull decoposition (while not throwing any error) is a
                    # good indicator of the level of fucked-upness of an asset. For now, we simply discard the asset.
                    os.rename(osp.join(obj_dir, params.VHACD_OUTPUTS[0]), osp.join(obj_dir, f'{obj_name}_{cvx_dir}.obj'))
                    os.remove(osp.join(obj_dir, params.VHACD_OUTPUTS[1]))
                elif acd_args.decompose_method == 'none':
                    # simply copy the STL
                    out_filename = osp.join(obj_dir, f'{obj_name}_{cvx_dir}.obj')
                    mesh.export(out_filename)
                    shutil.copy(obj_filename, osp.join(cvx_path, f'{obj_name}000.stl'))
                    decomposed = True
                elif acd_args.decompose_method == 'skip':
                    decomposed = False
                else:
                    raise ValueError(f'{obj_dir} decompose method {acd_args.decompose_method} is invalid')
            
            print(f"Removing {obj_filename}")
            os.remove(obj_filename)
            
        if decomposed:
            xyz, quat = utils.pose_rh_to_lh(xyz, quat)
            cls.assemble_mesh((obj_dir, output_folder, cvx_dir, xyz, quat, decompose_in_bodies))
        else:  # delete whole directory
            shutil.rmtree(obj_dir)
        return True

    
    def _process_assets(self):
        if not self.rerun:
            shutil.rmtree(self.output_dir, ignore_errors=True)
        
        # Select relevant objects in the scene (i.e. discard collision meshes, animations and so on...)
        objects = [obj for obj in bpy.context.scene.collection.all_objects if (obj.parent is None)]
        
        decompose_convex_args = []
        assemble_args = []
        for obj in objects:
            object_name = obj.name.split('.')[0]
            obj_dir = os.path.join(self.output_dir, object_name)
            obj_info = self.actors_information[object_name]

            # read joints info to augment it
            joints_info = {}
            if self.object_has_joint[obj.name.split('.')[0]]:
                filename = os.path.join(self.output_dir, '..', 'ue_export', f'{object_name}_joints.json')
                with open(filename, 'r') as f:
                    joints_info = json.load(f)
                joints_info = {j['name']: j for j in joints_info}

            # traverse the tree, whose leaves will be meshes. Collect leaves and their info for decomposition.
            leaves = []
            nodes_info = {}
            # each element is (bpy object, directory_path, parent transform, scale_factor, group_id, is_root)
            # parent transform is reset to I at every joint. It is also I for the actor root
            q = deque([(obj, obj_dir, np.eye(4), np.ones(3), object_name, True), ])
            while len(q):
                o, dir, T_parent, scale_factor, group_id, is_root = q.popleft()
                name = o.name.split('.')[0]
                xyz, quat = get_relative_pose_and_center(o, scale_factor)
                T_o = T_parent @ utils.xyzquat_to_T(xyz, quat)
                xyz, quat = utils.T_to_xyzquat(T_o)
                if o.children == ():  # StaticMeshComponent with geometry, or PhysicsConstraintComponent (i.e. joint)
                    if (o.type == 'MESH') and (o.data.name != 'SM_Dummy'):  # StaticMeshComponent with geometry
                        if name in nodes_info:
                            raise AssertionError(f'repeated mesh {name}')
                        nodes_info[name] = {'pos': xyz, 'quat': quat}
                        os.makedirs(dir, exist_ok=True)
                        decompose_method = obj_info['geoms'][name]['decompose_method']
                        leaves.append((dir, o, xyz, quat, decompose_method, group_id))
                    elif o.type == 'EMPTY':  # PhysicsConstraintComponent (i.e. joint)
                        joint_name = utils.name_in_names(name, joints_info.keys())
                        if joint_name is not None:
                            joints_info[joint_name]['pos'], joints_info[joint_name]['quat'] = xyz, quat
                else:  # StaticMeshComponent without any geometry (meant for grouping)
                    if name in nodes_info:
                        raise AssertionError(f'repeated mesh {name}')
                    nodes_info[name] = {'pos': xyz, 'quat': quat}
                    os.makedirs(dir, exist_ok=True)
                    if is_root or any([j['child'] == name for j in joints_info.values()]):
                        T_parent = np.eye(4)
                    else:
                        T_parent = np.copy(T_o)
                    for child in o.children:
                        child_name = child.name.split('.')[0]
                        if child_name in obj_info['geoms']:
                            group_id = obj_info['geoms'][child_name]['decompose_group']
                            group_id = f'{name}_{group_id}'
                        else:
                            group_id = None
                        q.append(
                            (child, osp.join(dir, child_name), T_parent, scale_factor*np.array(o.scale), group_id, False)
                        )

            # convert parent_T_joint to child_T_joint (which is what MuJoCo understands)
            for joint_info in joints_info.values():
                pTj = utils.xyzquat_to_T(joint_info['pos'], joint_info['quat'])
                pTc = utils.xyzquat_to_T(nodes_info[joint_info['child']]['pos'], nodes_info[joint_info['child']]['quat'])
                cTj = np.linalg.inv(pTc) @ pTj
                joint_info['pos']  = cTj[:3, 3].tolist()
                joint_info['quat'] = txq.mat2quat(cTj[:3, :3]).tolist()

            # group the children by group_id
            geom_groups = defaultdict(list)
            for leaf in leaves:
                geom_groups[leaf[-1]].append(leaf[:-1])
            
            joint_children = [object_name, ]
            joint_children.extend([j['child'] for j in joints_info.values()])
            for geom_group in geom_groups.values():
                leaf_dir, leaf, xyz, quat, decompose_method = geom_group[0]
                leaf_name = leaf.name.split('.')[0]
                
                # merge the geoms
                for l in geom_group[1:]:
                    l[1].select_set(True)
                    shutil.rmtree(l[0])
                leaf.select_set(True)
                bpy.context.view_layer.objects.active = leaf
                bpy.ops.object.join()
                
                if leaf_name in joint_children:
                    # zero out the transform w.r.t. parent, because this mesh's include file will be placed under
                    # an appropriately transformed body
                    xyz = [0.0, 0.0, 0.0, 0.0]
                    quat = [1.0, 0.0, 0.0, 0.0]
                
                # Calculate the number of faces in the mesh
                poly_count = len(leaf.data.polygons)
                print("Polygon count: ", poly_count)
                
                # Get the adaptive decimation ratio
                decimation_ratio = decimation_lookup(poly_count)
                    
                # Add the Decimate modifier to the selected object
                print("Applying decimation with a ratio: ", decimation_ratio)
                decimate_modifier = leaf.modifiers.new(name='Decimate', type='DECIMATE')
                decimate_modifier.ratio = decimation_ratio

                # Exporting in stl.
                stl_filepath = osp.join(leaf_dir, f"{leaf_name}.stl")
                if (not self.rerun) or (not osp.isfile(stl_filepath)):
                    print("Exporting STL file...")
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
                bpy.ops.object.select_all(action='DESELECT')

                cvx_args = params.ACD_Args(decompose_method, params.CoacdArgs(), params.VhacdArgs())
                decompose_convex_args.append(
                    (stl_filepath, self.output_dir, self.scene_path, "cvx", cvx_args, xyz, quat, False, self.rerun)
                )
            assemble_args.append(
                (object_name, obj_dir, self.output_dir, nodes_info, joints_info, obj_info['moving'],
                 f'.{obj_info["root_component_name"]}'),
            )
        # with mp.Pool(self.n_workers) as p:
        #     p.map(self.decompose_convex, decompose_convex_args)
        # with mp.Pool(self.n_workers) as p:
        #     p.map(assemble_articulated_object_files, assemble_args)
        print("Convex solid decomposition...")
        i = 0
        while True:
            start_idx = i * self.n_workers
            end_idx   = (i+1) * self.n_workers
            with mp.Pool(self.n_workers) as p:
                p.map(self.decompose_convex, decompose_convex_args[start_idx : end_idx])
            i = i + 1
            if end_idx >= len(decompose_convex_args):
                break
        print("Assembly...")
        i = 0
        while True:
            start_idx = i * self.n_workers
            end_idx   = (i+1) * self.n_workers
            with mp.Pool(self.n_workers) as p:
                p.map(self.assemble_object_with_joints, assemble_args[start_idx : end_idx])
            i = i + 1
            if end_idx >= len(assemble_args):
                break
    
    
    def export(self):
        # Deselect all objects
        bpy.ops.object.select_all(action='DESELECT')
        
        # Create a directory that will contain the decomposed assets
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Asset convex decomposition
        # articulated objects should be processed first
        self._process_assets()

        # Generate a MuJoCo xml file
        self.generate_scene()
