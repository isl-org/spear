import argparse
import numpy as np
import pandas as pd
import h5py

import mayavi.mlab # https://docs.enthought.com/mayavi/mayavi/installation.html

import os

from hypersim_load_obj_file_util import load_obj_file


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv_file", type=str, required=True)
    parser.add_argument("--obj_file", type=str)
    parser.add_argument("--hdf5_dir", type=str)
    parser.add_argument("--save_as_hdf5", action="store_true")
    parser.add_argument("--hdf5_save_folder_name", type=str, default="navmesh_export")
    args = parser.parse_args()

    if args.obj_file is None:
        
        assert args.hdf5_dir is not None, "input either obj file or hdf5 dir"

        # load hdf5
        with h5py.File(os.path.join(args.hdf5_dir, "vertices.hdf5"), "r") as f: mesh_vertices = f["dataset"][:]
        with h5py.File(os.path.join(args.hdf5_dir, "faces_vi.hdf5"), "r") as f: mesh_faces_vi = f["dataset"][:]
        with h5py.File(os.path.join(args.hdf5_dir, "faces_oi.hdf5"), "r") as f: mesh_faces_oi = f["dataset"][:]

    else:

        # load obj file
        isim_obj_file = load_obj_file(args.obj_file)
        mesh_vertices = isim_obj_file.vertices
        mesh_faces_vi = isim_obj_file.faces_vi
        mesh_faces_oi = isim_obj_file.faces_oi

        if args.save_as_hdf5:
            
            assert args.hdf5_dir is not None, "input hdf5_dir param to save as hdf5 files"

            hdf5_files_path = os.path.join(args.hdf5_dir, args.hdf5_save_folder_name)
            print(f"INFO: saving hdf5 files in {hdf5_files_path}")
            
            if not os.path.exists(hdf5_files_path):
                os.makedirs(hdf5_files_path)
            
            # save as hdf5 for easier loading next time
            with h5py.File(os.path.join(hdf5_files_path, f"vertices.hdf5"),  "w") as f: f.create_dataset("dataset", data=isim_obj_file.vertices)
            with h5py.File(os.path.join(hdf5_files_path, f"faces_oi.hdf5"),  "w") as f: f.create_dataset("dataset", data=isim_obj_file.faces_oi)
            with h5py.File(os.path.join(hdf5_files_path, f"faces_vi.hdf5"),  "w") as f: f.create_dataset("dataset", data=isim_obj_file.faces_vi)
    
    # read obj file generated by UE
    mesh_opacity = 0.2
    
    mesh_faces_oi_unique = np.unique(mesh_faces_oi)
    color_vals_unique = np.arange(mesh_faces_oi_unique.shape[0])
    np.random.shuffle(color_vals_unique)
    mesh_color_vals = np.zeros((mesh_vertices.shape[0]))
    for oi,color_val in zip(mesh_faces_oi_unique,color_vals_unique):
        mesh_color_vals[ mesh_faces_vi[ oi == mesh_faces_oi ].ravel() ] = color_val

    mayavi.mlab.triangular_mesh(-mesh_vertices[:,0], -mesh_vertices[:,2], mesh_vertices[:,1], mesh_faces_vi, scalars=mesh_color_vals, representation="surface", opacity=mesh_opacity)

    # read csv file from sampling random poses from navmesh
    df = pd.read_csv(args.csv_file, delimiter=',')
    v_x = df.iloc[:,0].to_numpy()#[::2]
    v_y = df.iloc[:,1].to_numpy()#[::2]
    v_z = df.iloc[:,2].to_numpy()#[::2]
    mayavi.mlab.points3d(v_x, v_y, v_z, opacity=1.0, scale_factor=1.2)

    mayavi.mlab.show()
    
    
# NOTES
# When .obj is exported via the UEEditor, this has different axes convetion from when exported with navmesh->GetGenerator()->ExportNavigationData()
# To overlay .obj exported from UEEditor and PCD data obtained from sampling points from navmesh, we need to do swap y and z axes of the mesh.
# To overlay .obj exported from navmesh and PCD data obtained from sampling points from navmesh, we need to swap y and z axes, and negate x and new y axes.
# Alternate: To overlay .obj exported from navmesh and PCD data obtained from sampling points from navmesh, instead of changing mesh axes, we can interpret the PCD data as follows;
#   v_x = -df.iloc[:,0].to_numpy()
#   v_y =  df.iloc[:,2].to_numpy()
#   v_z = -df.iloc[:,1].to_numpy()
