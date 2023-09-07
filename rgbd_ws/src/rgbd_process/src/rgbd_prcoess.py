#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pcl2

import cv2
import numpy as np
import sys
import sensor_msgs.point_cloud2 as pc2
import std_msgs
import geometry_msgs
from geometry_msgs.msg import PoseStamped 
np.set_printoptions(threshold=sys.maxsize)
import tf2_ros
import tf2_geometry_msgs
import open3d as o3d
from visualization_msgs.msg import Marker 

from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
from detectron2.data import MetadataCatalog



class ObjectDetection:

    def __init__(self):
        self.tf_buffer=tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge=CvBridge()
        # self.cfg = get_cfg()
        # self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        # self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
        # self.cfg.MODEL.WEIGHTS =model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
        # self.predictor = DefaultPredictor(self.cfg)
        self.camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)  
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.marker_publisher = rospy.Publisher('mean_marker', Marker, queue_size=1)
        self.pose_publisher = rospy.Publisher('/ComputedCentroid', PoseStamped, queue_size=10)
        # self.point_cloud_publisher = rospy.Publisher('point_cloud_made', PointCloud2, queue_size=1)
        

    def camera_info_callback(self, data):
        self.camera_intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.camera_intrinsic.set_intrinsics(
            data.width, data.height,
            data.K[0], data.K[4], data.K[2], data.K[5]
        )
    def depth_callback(self,data):
        self.rgbd_image=self.bridge.imgmsg_to_cv2(data,"passthrough").astype(np.uint16)

    def image_callback(self,data):
        rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # output = self.predictor(rgb_image)
        # unique_class_indices = np.unique(output['instances'].pred_classes.cpu().numpy())
        # print("Unique Class Indices:", unique_class_indices)
        # mask_output=output['instances'][output['instances'].pred_classes ==39].pred_masks
        # mask=list(mask_output)[0].detach().cpu().numpy()
        # segmentation=cv2.cvtColor((mask).astype(np.uint8)*255,cv2.COLOR_GRAY2BGR)
        # segmentation = segmentation[:,:,0] 
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        _, thresholded = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY_INV)
        # cv2.imshow("Segmented Image", thresholded)
        # cv2.waitKey(0)
        mask = np.where(thresholded == 255, 255, 0).astype(np.uint8)
        masked_rgbd_image = cv2.bitwise_and(self.rgbd_image, self.rgbd_image, mask=mask)
        
        if self.camera_intrinsic is not None:
            # Create the RGBDImage object
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(rgb_image),
                o3d.geometry.Image(masked_rgbd_image),
                depth_trunc=5.0, convert_rgb_to_intensity=False
            )
            
            # Create the point cloud
            point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd, self.camera_intrinsic
            )

            # Visualize the point cloud
            # o3d.visualization.draw_geometries([point_cloud])
            point_cloud_numpy = np.asarray(point_cloud.points)
            mean = np.mean(point_cloud_numpy, axis=0)
            print(mean)
            # header=std_msgs.msg.Header()
            # header.stamp=rospy.Time.now()
            # header.frame_id="camera_depth_optical_frame"
            # point_cloud_msg=pcl2.create_cloud_xyz32(header,point_cloud_numpy)
            # header=rospy.Header()
            # header.frame_id="camera_depth_optical_frame"
          

            # Publish the PointCloud2 message
            # self.point_cloud_publisher.publish(point_cloud_msg)
    



            transform=self.tf_buffer.lookup_transform(
                target_frame="panda_link0",
                source_frame="camera_depth_optical_frame",
                time=rospy.Time(0),
                timeout=rospy.Duration(1.0)
            )
            transformed_mean = tf2_geometry_msgs.do_transform_point(
                geometry_msgs.msg.PointStamped(
                        header=std_msgs.msg.Header(frame_id="camera_depth_optical_frame"),
                        point=geometry_msgs.msg.Point(x=mean[0], y=mean[1], z=mean[2])
                ),
                transform
            )            
            transformed_mean.point.z+=0.14
            # transformed_mean.point.x-=0.0515
            transformed_mean.point.x-=0.0616
            # transformed_mean.point.y=-0.01
            # transformed_mean.point.y-=0.075
            
            marker = Marker()
            marker.header.frame_id = "panda_link0"  # Set the appropriate camera frame
            marker.header.stamp = rospy.Time.now()
            marker.id = 0  # Keep the marker ID constant
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = transformed_mean.point.x
            marker.pose.position.y = transformed_mean.point.y
            marker.pose.position.z = transformed_mean.point.z
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.marker_publisher.publish(marker)

            pose_msg=PoseStamped()
            pose_msg.header.frame_id="panda_link0"
            pose_msg.pose.position.x = transformed_mean.point.x
            pose_msg.pose.position.y = transformed_mean.point.y
            pose_msg.pose.position.z = transformed_mean.point.z
            self.pose_publisher.publish(pose_msg)


        
rospy.init_node('ObjectDetection',anonymous=True)
od=ObjectDetection()
rospy.spin()


 