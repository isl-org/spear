import argparse
from collections import deque
from exporter_base import ExporterBase
from itertools import combinations
import mujoco
import multiprocessing as mp
import os
import utils
import xml.etree.ElementTree as ET


osp = os.path


class MujocoExporter(ExporterBase):
    def __init__(self, scene_path: str, n_workers: int, rerun: bool, include_objects: str) -> None:
        super(MujocoExporter, self).__init__(scene_path, n_workers, rerun, include_objects)
        self.output_dir = osp.join(self.scene_path, "mujoco_scene")

    
    @classmethod
    def assemble_mesh(cls, args):
        object_dir, output_dir, cvx_folder, xyz, quat, decompose_in_bodies = args
        print("populating xml file")
        _, body_name = os.path.split(object_dir)
        
        prefix = object_dir.replace(output_dir, '')
        if prefix[0] == osp.sep:
            prefix = prefix[1:]
        prefix = prefix.replace(osp.sep, '.')

        # write assets file
        root = ET.Element('mujoco', {'model': body_name})
        object_path = object_dir.replace(output_dir, '')
        if object_path[0] == osp.sep:
            object_path = object_path[1:]
        
        _, _, mesh_objects = next(os.walk(osp.join(object_dir, cvx_folder)))
        for mesh_object in sorted(mesh_objects):
            split_file_name = osp.basename(mesh_object)
            split_name, ext = osp.splitext(split_file_name)
            if (ext != '.stl') and (ext != '.obj'):
                continue
            ET.SubElement(root, 'mesh',
                        {'file': osp.join(object_path, cvx_folder, split_file_name), 'name': f'{prefix}.{split_name}'}
            )
        
        filename = osp.join(object_dir, cvx_folder, f'{body_name}_assets.xml')
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(filename)
        print(f'{filename} written')

        # write body file
        root = ET.Element('mujoco', {'model': body_name})
        file_name = osp.splitext(object_dir)
        # TODO(samarth) check if both cases are same
        if decompose_in_bodies:
            _, _, mesh_objects = next(os.walk(osp.join(object_dir, cvx_folder)))
            for mesh_object in sorted(mesh_objects):
                file_name = os.path.splitext(osp.basename(mesh_object))
                if (file_name[1] != '.stl') and (file_name[1] != '.obj'):
                    continue
                if not file_name[0].startswith("VolumeError"):
                    body_attributes = {
                        'name': file_name[0],
                        'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                        'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
                    }
                    # body = ET.SubElement(root, 'body', body_attributes)
                    geom_attributes = {
                        'name': f'{prefix}.{file_name[0]}_vis',
                        'mesh': f'{prefix}.{file_name[0]}',
                        'class': 'visual',
                        'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                        'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}', 
                    }
                    ET.SubElement(root, 'geom', geom_attributes)
                    geom_attributes = {
                        'name': f'{prefix}.{file_name[0]}_col',
                        'mesh': f'{prefix}.{file_name[0]}',
                        'class': 'collision',
                        'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                        'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}', 
                    }
                    ET.SubElement(root, 'geom', geom_attributes)
        else:
            body_attributes = {
                'name': body_name,
                'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
            }
            # body = ET.SubElement(root, 'body', body_attributes)
            # ET.SubElement(body, 'geom', {'name': f'{body_name}_vis', 'mesh': f'{body_name}_cvx', 'class': 'visual'})
            _, _, mesh_objects = next(os.walk(osp.join(object_dir, cvx_folder)))
            for mesh_object in sorted(mesh_objects):
                file_name = os.path.splitext(osp.basename(mesh_object))
                if ('.stl' not in file_name[1]) and ('.obj' not in file_name[1]):
                    continue
                if not file_name[0].startswith("VolumeError"):
                    geom_attributes = {
                        'name': f'{prefix}.{file_name[0]}_vis',
                        'mesh': f'{prefix}.{file_name[0]}',
                        'class': 'visual',
                        'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                        'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
                    }
                    ET.SubElement(root, 'geom', geom_attributes)
                    geom_attributes = {
                        'name': f'{prefix}.{file_name[0]}_col',
                        'mesh': f'{prefix}.{file_name[0]}',
                        'class': 'collision',
                        'pos': f'{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}',
                        'quat': f'{quat[0]:.5f} {quat[1]:.5f} {quat[2]:.5f} {quat[3]:.5f}',
                    }
                    ET.SubElement(root, 'geom', geom_attributes)
        filename = osp.join(object_dir, cvx_folder, f'{body_name}_body.xml')
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)
        tree.write(filename)
        print(f'{filename} written')
        print(f"Done !")
        print(f"----------------------------------------------------------------------------")
        

    @classmethod
    def assemble_object_with_joints(cls, args):
        obj_name, obj_dir, output_dir, nodes_info, joints_info, moving, parent_name_suffix = args
        full_obj_name = f'{obj_name}{parent_name_suffix}'
        root = ET.Element('mujoco')
        asset = ET.SubElement(root, 'asset')
        worldbody = ET.SubElement(root, 'worldbody')
        contact = ET.SubElement(root, 'contact')

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
            joint = list(filter(lambda j: (j['parent'] == parent_name) and (j['child'] == name), joints_info.values()))
            
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
                        'type': joint[0]['type'],
                        'pos': '{:.5f} {:.5f} {:.5f}'.format(*pos),
                        'axis': '{:.5f} {:.5f} {:.5f}'.format(*axis),
                        'range': '{:.5f} {:.5f}'.format(*joint[0]['range']),
                        'ref': '{:.5f}'.format(joint[0]['ref'])
                    }
                    ET.SubElement(body, 'joint', joint_attributes)
                elif moving:
                    ET.SubElement(body, 'freejoint', {'name': f'{obj_name}.freejoint'})
                js = list(filter(lambda j: j['parent'] == name, joints_info.values()))
                if len(js) == 0:
                    leaf_dirs.add(dir)
                dir2bodyname[dir] = body_name
                geom_parent = body
                name_prefix = f'{name_prefix}{name}{parent_name_suffix}.'
                parent_name_suffix = ''
            elif len(joint) >= 2:
                raise AssertionError(f'More than one joint found with parent {parent_name} and child {name}')
            
            # check if body has its geom(s)
            cvx_dir = osp.join(dir, 'cvx')
            if osp.isdir(cvx_dir):
                include_dir = cvx_dir.replace(output_dir, '')
                if include_dir[0] == '/':
                    include_dir = include_dir[1:]
                ET.SubElement(geom_parent, 'include', {'file': osp.join(include_dir, f'{name}_body.xml')})
                ET.SubElement(asset, 'include', {'file': osp.join(include_dir, f'{name}_assets.xml')})
            
            # explore children
            _, child_dirs, _ = next(os.walk(dir))
            child_dirs = [osp.join(dir, child_dir) for child_dir in sorted(child_dirs) if child_dir != 'cvx']
            q.extendleft([(child_dir, dir, geom_parent, name_prefix) for child_dir in child_dirs])
            if len(child_dirs):
                sibling_dir_groups.append(child_dirs)

        # contact exclude tags for leaf sibling bodies
        for sibling_dir_group in sibling_dir_groups:
            sdg = [s for s in sibling_dir_group if s in leaf_dirs]
            for body_pair in combinations(sdg, 2):
                ET.SubElement(contact, 'exclude', {'body1': dir2bodyname[body_pair[0]], 'body2': dir2bodyname[body_pair[1]]})
            for dir in sdg:
                ET.SubElement(contact, 'exclude', {'body1': dir2bodyname[dir], 'body2': full_obj_name})
        
        # write MJCF
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)
        filename = os.path.join(obj_dir, f'{obj_name}.xml')
        tree.write(filename)
        print(f'{filename} written')


    @classmethod
    def check_mesh_validity(cls, filename: str) -> bool:
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


    def generate_scene(self):
        template_root = ET.parse(osp.join('mujoco_export_pipeline', 'scene_include.xml')).getroot()
        root = ET.Element('mujoco', {'model': osp.split(self.scene_path)[1]})
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
    parser.add_argument('--scene_path', required=True, help='relative path to the scene folder')
    parser.add_argument('--rerun', action='store_true', help='re-run the export without running decomposition')
    parser.add_argument('--objects', help='comma separated', default=None)
    parser.add_argument('-n', type=int, help='number of parallel workers to use', default=mp.cpu_count()-1)
    args = parser.parse_args()
    
    exporter = MujocoExporter(osp.expanduser(args.scene_path), args.n, args.rerun, args.objects)
    exporter.export()