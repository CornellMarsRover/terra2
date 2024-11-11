import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import pcl
from pcl import PointCloud
import pcl.pcl_visualization
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header

def EuclideanCluster(Node):
    def __init__(self):
        super().__init__('euclidean_cluster')

        self.pc2_sub = self.create_subscription(PointCloud2, '/zed/point_cloud', self.callback, 10)


    def callback(self, msg: PointCloud2):
        self.get_logger().info('Received PointCloud2 data')

        cloud = pc2.read.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        cloud_np = np.array(list(cloud))
        cloud_pcl = pcl.PointCloud().from_array(cloud_np.astype(np.float32))

        self.cluster(cloud_pcl)

def perform_clustering(self, pcl_cloud: pcl.PointCloud):
        tree = pcl_cloud.make_kdtree()
        ec = pcl_cloud.make_EuclideanClusterExtraction()

        ec.set_ClusterTolerance(0.05)

        ec.set_MinClusterSize(100)
        ec.set_MaxClusterSize(25000)

        ec.set_SearchMethod(tree)

        cluster_indices = ec.Extract()

        self.get_logger().info(f"Found {len(cluster_indices)} clusters")

        for i, indices in enumerate(cluster_indices):
            self.get_logger().info(f"Cluster {i + 1} has {len(indices)} points")
            cluster_points = pcl_cloud.extract(indices)


def main(args=None):
    rclpy.init(args=args)
    node = EuclideanCluster()
    rclpy.spin()
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
