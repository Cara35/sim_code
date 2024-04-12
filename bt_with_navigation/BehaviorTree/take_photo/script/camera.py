#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import base64
from cv_bridge import CvBridge
import cv2
import time
from std_msgs.msg import String

from bt_task_msgs.srv import TakePhoto,TakePhotoResponse

# 保存路径
save_path = '/home/agilex/behaviortree_ws/src/img/'    # 已经建立好的存储cam0 文件的目录

class TakePhotoService:
    def __init__(self):
        rospy.init_node('take_photo_server', anonymous=True)
        # publish
        self.base64_pub = rospy.Publisher('base64_data', String, queue_size=10)
        # service
        self.take_photo_service = rospy.Service('/take_photo_service', TakePhoto, self.take_photo_service_CB)
        
        self.bridge = CvBridge()

    def take_photo_service_CB(self, req):
            rospy.loginfo("Capturing and publishing a new image...")

            image_name = "captured_image_{}.jpg".format(int(time.time()))  # 使用时间戳作为文件名

            current_image = None
            try:
                # 阻塞等待接收 /camera_01/color/image_raw 话题的消息，并返回类型为 sensor_msgs.msg.Image 的消息对象。
                # 缺点:指定的话题不存在或没有发布消息，那么程序将一直被堵塞在这，不会执行后续的代码,加上timeout=5后，超过五秒还未收到则反馈ERROR。
                current_image = self.bridge.imgmsg_to_cv2(rospy.wait_for_message("/"+req.camera_number+"/color/image_raw", Image, timeout=3), 'bgr8')
                cv2.imwrite(save_path + image_name, current_image)
            except rospy.exceptions.ROSException as e:
                rospy.logerr("Timeout waiting for image message: {}".format(e))
                return TakePhotoResponse(success=False, message="Timed out waiting for image message" , code=10101)
            except CvBridgeError as e:
                rospy.logerr(e)
                return TakePhotoResponse(success=False, message="Failed to capture image", code=10102)
            
            # 处理图像，在中心点创建0点，并绘制直线
            height, width, _ = current_image.shape
            center_x, center_y = width // 2, height // 2

            # 在中心绘制0点
            cv2.circle(current_image, (center_x, center_y), 5, (0, 255, 0), -1)

            # 绘制水平和垂直直线
            cv2.line(current_image, (center_x, 0), (center_x, height), (255, 0, 0), 2)  # 垂直线
            cv2.line(current_image, (0, center_y), (width, center_y), (255, 0, 0), 2)  # 水平线

            # 保存带有直线的图像
            cv2.imwrite(save_path + "annotated_" + image_name, current_image)
            
            # image to base64
            image_data = cv2.imencode('.jpg', current_image)[1].tostring()
            base64_data = base64.b64encode(image_data)

            # Pub base64 data
            self.base64_pub.publish(String(data=base64_data))

            return TakePhotoResponse(success=True, message="Image captured and published successfully", code=10103)
            
if __name__ == '__main__':
    try:
        rospy.loginfo("Take photo service start !")
        take_photo_service = TakePhotoService()
        rospy.spin()
    except rospy.ROSInitException:
        pass
        