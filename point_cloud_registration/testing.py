import time
import numpy as np
import open3d as o3d
import transformations as tfs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Transform, Vector3
from . import utils

CONTROL_POINTS = [(-1, 7, 0), (-1, 5, 0), (1, 5, 0), (1, 7, 0), (-0.5, 6.5, 2), (0, 5.5, 2), (0.5, 6.5, 2)]
TRANSLATE_POINTS = [(-1, 4, 0), (-1, 2, 0), (1, 2, 0), (1, 4, 0), (-0.5, 3.5, 2), (0, 2.5, 2), (0.5, 3.5, 2)]
ROTATE_POINTS = [(7, 1, 0), (5, 1, 0), (5, -1, 0), (7, -1, 0), (6.5, 0.5, 2), (5.5, 0, 2), (6.5, -0.5, 2)]
TRANSLATE_ROTATE_POINTS = [(5, 0, 0), (3, 0, 0), (3, -2, 0), (5, -2, 0), (4.5, -0.5, 2), (3.5, -1, 2), (4.5, -1.5, 2)]

POINTS = [CONTROL_POINTS, TRANSLATE_POINTS, ROTATE_POINTS, TRANSLATE_ROTATE_POINTS]
POSITION = [
    Transform(),
    Transform(translation=Vector3(y=-3.0)),
    Transform(rotation=Quaternion(z=0.707, w=0.707)),
    Transform(translation=Vector3(x=1.0, y=-2.0), rotation=Quaternion(z=0.707, w=0.707))
]


class TestingNode(Node):
    def __init__(self):
        super().__init__("testing")
        self.points = set()
        self.process_frame(0)
        self.process_frame(1)
        self.save_pcd("translate")
        self.points = set()
        self.process_frame(0)
        self.process_frame(2)
        self.save_pcd("rotate")
        self.points = set()
        self.process_frame(0)
        self.process_frame(3)
        self.save_pcd("translate_rotate")
    
    def get_transform_matrices(self, transform):
        Vq = transform.rotation
        Vq = [Vq.w, Vq.x, Vq.y, Vq.z]
        Vt = transform.translation
        Vt = [Vt.x, Vt.y, Vt.z]
        return np.around(tfs.quaternion_matrix(Vq), 5), np.around(tfs.translation_matrix(Vt), 5)

    def process_frame(self, iteration):
        self.get_logger().info(f"Processing frame...")
        Mr, Mt = self.get_transform_matrices(POSITION[iteration])
        Ms = tfs.scale_matrix(-1)

        pcd_data = np.array(POINTS[iteration])
        P = np.ones((pcd_data.shape[0], 4))  # add the fourth column
        P[:, :-1] = pcd_data
        
        # P = np.around(tfs.concatenate_matrices(Mt, Ms, Mr, Mr.T, P.T), 3).T
        P = np.around(tfs.concatenate_matrices(Mt, Ms, np.dot(Mr, P.T)), 3).T

        # tuples are hashable objects and will cause collisions when added to a set
        new_points = list(map(lambda t: (t[0], t[1], t[2]), P))
        self.points.update(new_points)

        self.get_logger().info(f"Frame processed")


    def save_pcd(self, suffix):
        try:
            self.get_logger().info(f"Saving pointcloud ({len(self.points)} points)...")
            out_pcd = o3d.geometry.PointCloud()
            out_pcd.points = o3d.utility.Vector3dVector(list(self.points))
            timestr = time.strftime("%Y%m%d%H%M%S")
            o3d.io.write_point_cloud(f"./{timestr}_{suffix}.pcd", out_pcd)
            self.get_logger().info(f"Saved pointcloud {timestr}_{suffix}.pcd")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    get_pcd_node = TestingNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

