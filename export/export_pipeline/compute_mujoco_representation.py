import argparse
from collections import deque
from itertools import combinations
import json
import mujoco
import numpy as np
import os
import transforms3d.quaternions as txq
import utils
import xml.etree.ElementTree as ET
import yaml


osp = os.path


joint_type_mapping = {
    'prismatic': 'slide',
    'revolute' : 'hinge',
}


def check_mesh_validity(filename: str) -> bool:
    """
    filename: STL file name
    """
    root = ET.Element('mujoco')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'mesh', {'file': filename, 'name': 'test_mesh_asset'})
    wbody = ET.SubElement(root, 'worldbody')
    body = ET.SubElement(wbody, 'body', {'name': 'test_body'})
    ET.SubElement(body, 'geom', {'name': 'test_mesh', 'mesh': 'test_mesh_asset'})
    try:
        m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        d = mujoco.MjData(m)
    except:
        print(f'!!!!!!!! invalid mesh {filename} !!!!!!!!')
        return False
    return True


class MuJoCoSceneAssembler(object):
    def __init__(self, scene_path: str, include_actors: tuple) -> None:
        with open(osp.join('export_pipeline', 'params.yaml'), 'r') as f:
            self.p: dict = yaml.safe_load(f)
        self.scene_name = osp.split(scene_path)[1]
        self.input_dir = osp.join(scene_path, self.p['common']['COLLISION_DIR_NAME'])
        self.output_dir = osp.join(scene_path, self.p['common']['PHYSICS_DIR_NAME'].format('mujoco'))
        self.include_objects = include_actors
        if not osp.isdir(self.input_dir):
            raise FileNotFoundError(f'Missing collision representation at {self.input_dir}')


    def run(self):
        filename = osp.join(self.input_dir, '..', self.p['common']['GLTF_SCENE_DIR_NAME'], 'actors_information.json')
        with open(filename, 'r') as f:
            actors_information = json.load(f)
        gltf_scene_dir = self.input_dir.replace(self.p['common']['COLLISION_DIR_NAME'], self.p['common']['GLTF_SCENE_DIR_NAME'])
        for object_name in sorted(next(os.walk(self.input_dir))[1]):
            if self.include_objects and (object_name not in self.include_objects):
                continue
            object_dir = osp.join(self.input_dir, object_name)
            filename = osp.join(object_dir, 'components.json')
            with open(filename, 'r') as f:
                components_info = json.load(f)
            static_components_info = components_info['static']
            
            try:
               filename = osp.join(gltf_scene_dir, f'{object_name}_joints.json')
               with open(filename, 'r') as f:
                   joints_info = json.load(f)
            except FileNotFoundError:
                joints_info = []
            
            # convert parent_T_joint to child_T_joint (which is what MuJoCo understands)
            for joint_info in joints_info:
                joint_name = joint_info['name']
                pTj = utils.xyzquat_to_T(
                    components_info['joints'][joint_name]['pos'],
                    components_info['joints'][joint_name]['quat']
                )
                child_xyz  = static_components_info[joint_info['child']]['pos']
                child_quat = static_components_info[joint_info['child']]['quat']
                pTc = utils.xyzquat_to_T(child_xyz, child_quat)
                cTj = np.linalg.inv(pTc) @ pTj
                joint_info['pos']  = cTj[:3, 3].tolist()
                joint_info['quat'] = txq.mat2quat(cTj[:3, :3]).tolist()
            joint_children = [object_name, ]
            joint_children.extend([j['child'] for j in joints_info])
            
            # TODO(samarth): remove the transform compositions, because they are not needed here
            # Traverse directory tree, and assemble the convex regions of meshes at leaves
            q = deque([(object_name, object_dir, np.eye(4)), ])
            while len(q):
                name, dir, T_parent = q.popleft()
                xyz, quat = static_components_info[name]['pos'], static_components_info[name]['quat']
                T_o = T_parent @ utils.xyzquat_to_T(xyz, quat)
                xyz, quat = utils.T_to_xyzquat(T_o)
                static_components_info[name]['pos'], static_components_info[name]['quat'] = xyz, quat

                for child_name in next(os.walk(dir))[1]:
                    if child_name == self.p['common']['CONVEX_DECOMPOSITION_DIR']:
                        child_dir = osp.join(dir, child_name)
                        for mesh_name in next(os.walk(child_dir))[1]:
                            mesh_dir = osp.join(child_dir, mesh_name)
                            i = static_components_info[name]['meshes'][mesh_name]
                            self.assemble_mesh(mesh_dir, i['pos'], i['quat'], i['decompose_method'])
                    else:
                        T_parent = np.eye(4) if child_name in joint_children else np.copy(T_o)
                        q.append((child_name, osp.join(dir, child_name), T_parent))
            self.assemble_object_with_joints(object_dir, static_components_info, joints_info,
                                             actors_information[object_name])
        self.generate_scene()
    
    
    def assemble_mesh(self, mesh_dir, xyz, quat, decompose_method):
        """
        Assembles the convex regions of a mesh
        """
        xyz, quat = utils.pose_rh_to_lh(xyz, quat)
        mesh_name = osp.split(mesh_dir)[1]
        path_prefix = mesh_dir.replace(self.input_dir, '')
        if path_prefix[0] == osp.sep:
            path_prefix = path_prefix[1:]
        output_dir = osp.join(self.output_dir, path_prefix)
        os.makedirs(output_dir, exist_ok=True)

        # write assets file
        root = ET.Element('mujocoinclude', {'model': mesh_name})
        asset_name_prefix = path_prefix.replace(osp.sep, '.')
        asset_path_prefix = osp.join('..', self.p['common']['COLLISION_DIR_NAME'], path_prefix, decompose_method)
        asset_names = []
        for mesh_filename in sorted(next(os.walk(osp.join(mesh_dir, decompose_method)))[2]):
            region_name, ext = osp.splitext(osp.basename(mesh_filename))
            if (ext != self.p['common']['CONVEX_DECOMPOSITION_EXT']) or (region_name == 'assembled'):
                continue
            asset_name = f'{asset_name_prefix}.{region_name}'
            asset_names.append(asset_name)
            asset_attributes = {
                'file': osp.join(asset_path_prefix, mesh_filename),
                'name': asset_name
            }
            ET.SubElement(root, 'mesh', asset_attributes)
        filename = osp.join(output_dir, 'assets.xml')
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(filename)
        print(f'{filename} written')

        # write body file
        root = ET.Element('mujocoinclude', {'model': mesh_name})
        for asset_name in asset_names:
            geom_attributes = {
                'name': f'{asset_name}_vis',
                'mesh': asset_name,
                'class': 'visual',
                'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
            }
            ET.SubElement(root, 'geom', geom_attributes)
            geom_attributes = {
                'name': f'{asset_name}_col',
                'mesh': asset_name,
                'class': 'collision',
                'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
            }
            ET.SubElement(root, 'geom', geom_attributes)
        filename = osp.join(output_dir, 'body.xml')
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(filename)
        print(f'{filename} written')
        

    def assemble_object_with_joints(self, obj_dir, nodes_info, joints_info, obj_info):
        obj_name = osp.split(obj_dir)[1]
        parent_name_suffix = f'.{obj_info["root_component_name"]}'
        full_obj_name = f'{obj_name}{parent_name_suffix}'
        root = ET.Element('mujocoinclude', {'model': obj_name})
        asset = ET.SubElement(root, 'asset')
        worldbody = ET.SubElement(root, 'worldbody')
        contact = ET.SubElement(root, 'contact')
        actuator = ET.SubElement(root, 'actuator')

        # elements are (directory, parent directoy, parent body tag, name prefix)
        q = deque([(obj_dir, None, worldbody, ''), ])
        sibling_dir_groups = []
        dir2bodyname = {}
        leaf_dirs = set()
        while len(q):
            dir, parent_dir, parent, name_prefix = q.popleft()
            parent_name = osp.split(parent_dir)[-1] if parent_dir is not None else None
            name = osp.split(dir)[-1]
            geom_parent = parent
            
            # check if body has a joint with parent
            joint = list(filter(lambda j: j['child'] == name, joints_info))
            
            if (len(joint) == 1) or (parent_name is None):
                body_name = f'{name_prefix}{name}{parent_name_suffix}'
                pos, quat = utils.pose_rh_to_lh(nodes_info[name]['pos'], nodes_info[name]['quat'])
                body_attributes = {
                    'name': body_name,
                    'pos': '{:f} {:f} {:f}'.format(*pos),
                    'quat': '{:f} {:f} {:f} {:f}'.format(*quat)
                }
                body = ET.SubElement(parent, 'body', body_attributes)
                if len(joint) == 1:
                    joint_name = f'{name_prefix}{joint[0]["name"]}{parent_name_suffix}'
                    pos, _ = utils.pose_rh_to_lh(joint[0]['pos'], (1, 0, 0, 0))
                    axis, _ = utils.pose_rh_to_lh(joint[0]['axis'], (1, 0, 0, 0))
                    joint_attributes = {
                        'name': joint_name,
                        'type': joint_type_mapping[joint[0]['type']],
                        'pos': '{:.5f} {:.5f} {:.5f}'.format(*pos),
                        'axis': '{:.5f} {:.5f} {:.5f}'.format(*axis),
                        'range': '{:.5f} {:.5f}'.format(*joint[0]['range']),
                        'ref': '{:.5f}'.format(joint[0]['ref']),
                        'damping': f'{self.p["mujoco"]["joint_kd"]:.5f}',
                    }
                    ET.SubElement(body, 'joint', joint_attributes)
                    actuator_attributes = {
                        'kp': f'{self.p["mujoco"]["joint_kp"]:.5f}',
                        'ctrllimited': 'true',
                        'ctrlrange': '{:.5f} {:.5f}'.format(*np.deg2rad(joint[0]['range'])),
                        'joint': joint_name,
                        'name': f'act:{joint_name}',
                    }
                    ET.SubElement(actuator, 'position', actuator_attributes)
                elif obj_info['simulate_physics']:
                    ET.SubElement(body, 'freejoint', {'name': f'{obj_name}.freejoint'})
                js = list(filter(lambda j: j['parent'] == name, joints_info))
                if len(js) == 0:
                    leaf_dirs.add(dir)
                dir2bodyname[dir] = body_name
                geom_parent = body
                name_prefix = f'{name_prefix}{name}{parent_name_suffix}.'
                parent_name_suffix = ''
            elif len(joint) >= 2:
                raise AssertionError(f'More than one joint found with parent {parent_name} and child {name}')
            
            this_sibling_dir_group = []
            for child_name in sorted(next(os.walk(dir))[1]):
                child_dir = osp.join(dir, child_name)
                if child_name == self.p['common']['CONVEX_DECOMPOSITION_DIR']:
                    for mesh_name in sorted(next(os.walk(child_dir))[1]):
                        mesh_dir = osp.join(child_dir, mesh_name)
                        path_prefix = mesh_dir.replace(self.input_dir, '')
                        if path_prefix[0] == osp.sep:
                            path_prefix = path_prefix[1:]
                        ET.SubElement(geom_parent, 'include', {'file': osp.join(path_prefix, 'body.xml')})
                        ET.SubElement(asset, 'include', {'file': osp.join(path_prefix, 'assets.xml')})
                else:
                    q.append((child_dir, dir, geom_parent, name_prefix))
                    this_sibling_dir_group.append(child_dir)
            if len(this_sibling_dir_group):
                sibling_dir_groups.append(this_sibling_dir_group)

        # contact exclude tags for leaf sibling bodies
        for sibling_dir_group in sibling_dir_groups:
            sdg = [s for s in sibling_dir_group if s in leaf_dirs]
            for body_pair in combinations(sdg, 2):
                ET.SubElement(contact, 'exclude', {'body1': dir2bodyname[body_pair[0]], 'body2': dir2bodyname[body_pair[1]]})
            for dir in sdg:
                ET.SubElement(contact, 'exclude', {'body1': dir2bodyname[dir], 'body2': full_obj_name})
        
        # write MJCF
        filename = osp.join(obj_dir, f'{obj_name}.xml')
        filename = filename.replace(self.input_dir, self.output_dir)
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(filename)
        print(f'{filename} written')


    def generate_scene(self):
        template_root = ET.parse(osp.join('export_pipeline', 'scene_include.xml')).getroot()
        root = ET.Element('mujoco', {'model': self.scene_name})
        ET.SubElement(root, 'include', {'file': 'objects.xml'})
        for t in template_root:
            root.insert(-1, t)
        tree = ET.ElementTree(root)
        ET.indent(tree, space='\t', level=0)
        filename = osp.join(self.output_dir, "scene.xml")
        tree.write(filename)
        print(f'{filename} written')

        root = ET.Element('mujoco')
        for body_name in sorted(next(os.walk(self.output_dir))[1]):
            if body_name == ".DS_Store":
                continue
            ET.SubElement(root, 'include', {'file': osp.join(body_name, f'{body_name}.xml')})
        tree = ET.ElementTree(root)
        ET.indent(tree, space='\t', level=0)
        filename = osp.join(self.output_dir, 'objects.xml')
        tree.write(filename)
        print(f'{filename} written')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--pipeline_dir', required=True)
    parser.add_argument('--actors', default=None)
    parser.add_argument('--scene_id', default=None)
    args = parser.parse_args()
    
    include_actors = tuple(args.actors.split(',')) if args.actors else ()

    for scene_name in sorted(next(os.walk(args.pipeline_dir))[1]):
        if (args.scene_id is not None) and (scene_name != args.scene_id):
            continue
        print(f'############# Scene {scene_name} ############')
        mse = MuJoCoSceneAssembler(osp.join(args.pipeline_dir, scene_name), include_actors)
        mse.run()