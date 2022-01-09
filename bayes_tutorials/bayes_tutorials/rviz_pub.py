import time
import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3


class VizMarkerPub(Node):

    def __init__(self):
        super().__init__("rviz_marker_publish_node")

        

        self._marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self._timer = self.create_timer(0.02, self.publish_callback)

    def get_text_args(self, idx, prob_value):

        pose = Pose()
        pose.position.x = -4.5+idx
        pose.position.y = 0.0
        pose.position.z = 0.15+prob_value*10
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        scale = Vector3()
        scale.x = 0.01
        scale.y = 0.01
        scale.z = 0.4

        color = ColorRGBA()
        color.r = 0.0
        color.g = 1.0
        color.b = 0.0
        color.a = 0.8

        return pose, scale, color

    def get_column_args(self, idx, prob_value):

        pose = Pose()
        pose.position.x = -4.5+idx
        pose.position.y = 0.0
        pose.position.z = (prob_value*10)/2
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        scale = Vector3()
        scale.x = 0.2
        scale.y = 0.01
        scale.z = prob_value*10

        color = ColorRGBA()
        color.r = 0.0
        color.g = 1.0
        color.b = 0.0
        color.a = 0.8

        return pose, scale, color

    def publish_callback(self):
        
        prob_value = 0.100000000
        
        for idx in range(10):

            text_pose, text_scale, text_color = self.get_text_args(idx, prob_value)

            text_marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=idx,
                lifetime=Duration(seconds=0.0).to_msg(),
                pose=text_pose,
                scale=text_scale,
                header=Header(frame_id='map'),
                color=text_color,
                text=str(prob_value)[:5]
            )
            self._marker_publisher.publish(text_marker)
            time.sleep(0.02)
            
            column_pose, column_scale, column_color = self.get_column_args(idx, prob_value)
            column_marker = Marker(
                type=Marker.CUBE,
                id=10+idx,
                lifetime=Duration(seconds=0.0).to_msg(),
                pose=column_pose,
                scale=column_scale,
                header=Header(frame_id='map'),
                color=column_color,
            )
            self._marker_publisher.publish(column_marker)
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = VizMarkerPub()

    rclpy.spin(cmd_vel_publisher)

    cmd_vel_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()