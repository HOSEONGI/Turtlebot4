import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import test.py
from visualization_msgs.msg import Marker
import geometry_msgs.msg 




class ImageProcessor(Node):

    flag = False

    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.qrinfo = []
        self.image_sub = self.create_subscription(
            Image, '/oakd/rgb/preview/image_raw', self.image_callback, qos_profile_sensor_data)


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            qrimage, decoded_text = self.detect_qr_code(cv_image)
            #edges, lines = self.detect_edges_and_lines(cv_image)

            if qrimage is not None:
                cv2.imshow('QR Code Detected', qrimage)
                cv2.waitKey(1)

        except Exception as e:
            print(e)

    def detect_qr_code(self, img):
        image = np.copy(img)
        det = cv2.QRCodeDetector()
        info, box_coordinates, _ = det.detectAndDecode(image)
        
        if (len(self.qrinfo) == 4):
            flag = True
            print(flag)
        
        
        if box_coordinates is None or info == "":
            pass
        else:
            if info not in self.qrinfo:
                self.qrinfo.append(info)
                print(self.qrinfo)

                marker = Marker()
                marker.header.frame_id = "map"
                marker.id = self.marker_id  
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                position = geometry_msgs.msg.Point()
                position.x = test.self.x
                position.y = test.self.y
                position.z = 0.0
                marker.pose.position = position
                #marker.pose.orientation = trans.transform.rotation
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                self.marker_publisher.publish(marker)
                

            if box_coordinates is not None:
                box_coordinates = [box_coordinates[0].astype(int)]
                n = len(box_coordinates[0])
                for i in range(n):
                    cv2.line(
                        image,
                        tuple(box_coordinates[0][i]),
                        tuple(box_coordinates[0][(i + 1) % n]),
                        (0, 255, 0),
                        3,
                    )

        return image, info

def main():
    rclpy.init()
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


