import argparse
import open3d as o3d
import os.path as osp

def visualize_ply(filename):
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f'Show {osp.basename(filename)}')
    # set_view(self.vis, dist=10, theta=80 * np.pi / 180)
    
    mesh = o3d.io.read_triangle_mesh(filename)
    # print(len(mesh.vertices))
    # mesh.compute_vertex_normals()
    # mesh.paint_uniform_color([i/255 for i in [112, 128, 144]])
    
    vis.add_geometry(mesh, True)

    vis.run()
    vis.destroy_window()

if __name__=="__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument("filename", type=str,  help = "The file to be visualized")

    args = argparser.parse_args()
    filename = args.filename
    filesuffix = filename.split('.')[-1]
    print(f"filename = {filename}")
    if filesuffix == 'ply':
        visualize_ply(filename)