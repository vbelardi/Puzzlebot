import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from ultralytics import YOLO
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class objectDetector(Node):
    def __init__(self):
        super().__init__('obj_cnn_node')

        self.valid_img = False
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        self.pubObj = self.create_publisher(String, '/objectName', 10)

        self.get_logger().info('offset detector node start')
        classNames = ['Derecha-pannels', 'Izquierda-pannels', 'Jaune-sema', 'Prio-pannels', 'Red-sema', 'Rondpoint-pannels', 'Stop-pannels', 'Toutdroit-pannels', 'Travaux-pannels','Vert-sema']
        self.model = YOLO('/home/puzzlebot/ros2_ws/src/basic_comms/  basic_comms/MODELO_cerveza.pt')


    def object_detector(self, frame):
        results = self.model(frame, stream=True)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                #x1, y1, x2, y2 = box.xyxy[0]
                #x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                #cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
                confidence = math.ceil((box.conf[0]*100))/100
                print(confidence)
                cls = int(box.cls[0])
                #org = [x1, y1]
                #font = cv2.FONT_HERSEY_SIMPLEX
                #fontScale = 1
                #color = (255, 0, 0)
                #thinkness = 2
                stringClass = self.classNames[cls]
                return stringClass
        return "none"

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            object_name = self.object_detector(cv_image)
            self.pubObj.publish(String(data=object_name))

        except:
            self.get_logger().info('neural network failed to read CAM')

def main(args=None):
    rclpy.init(args=args)
    cv_objDetector = objectDetector()
    rclpy.spin(cv_objDetector)
    cv_objDetector.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()












