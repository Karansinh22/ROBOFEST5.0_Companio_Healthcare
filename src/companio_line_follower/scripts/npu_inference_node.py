#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2

try:
    from hailo_platform import HEF, Device, VDevice, HailoStreamInterface, InferVStreams, ConfigureParams, InputVStreamParams, OutputVStreamParams, FormatType
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False
    rospy.logwarn("Hailo Platform SDK not found. Running in simulation mode.")

class NPUInferenceNode:
    def __init__(self):
        rospy.init_node('npu_inference_node', anonymous=True)
        
        # Parameters
        self.hef_path = rospy.get_param('~hef_path', '')
        self.input_width = rospy.get_param('~input_width', 640)
        self.input_height = rospy.get_param('~input_height', 480)
        
        # Publishers/Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.result_pub = rospy.Publisher('/ai/inference_results', Float32MultiArray, queue_size=10)
        
        self.bridge = CvBridge()
        
        # Hailo Initialization
        if HAILO_AVAILABLE and self.hef_path:
            self.init_npu()
        else:
            rospy.loginfo("NPU Inference node initialized in bypassed/simulation mode.")

    def init_npu(self):
        try:
            self.hef = HEF(self.hef_path)
            self.target = VDevice()
            configure_params = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
            self.network_group = self.target.configure(self.hef, configure_params)[0]
            
            self.input_vstream_params = InputVStreamParams.make_input_vstream_params(self.network_group, format_type=FormatType.UINT8)
            self.output_vstream_params = OutputVStreamParams.make_output_vstream_params(self.network_group, format_type=FormatType.FLOAT32)
            
            rospy.loginfo("Hailo NPU successfully initialized.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Hailo NPU: {e}")
            self.target = None

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Preprocessing (Resize for NPU)
            resized_image = cv2.resize(cv_image, (self.input_width, self.input_height))
            input_data = np.expand_dims(resized_image, axis=0).astype(np.uint8)
            
            # Run Inference
            if HAILO_AVAILABLE and hasattr(self, 'target') and self.target:
                with InferVStreams(self.network_group, self.input_vstream_params, self.output_vstream_params) as infer_pipeline:
                    input_dict = {self.hef.get_input_vstream_infos()[0].name: input_data}
                    with infer_pipeline.infer(input_dict) as infer_results:
                        # Extract results (Placeholder for specific model output processing)
                        # In this case, we'd process the output tensor based on the model used
                        output_name = self.hef.get_output_vstream_infos()[0].name
                        results = infer_results[output_name]
                        self.publish_results(results.flatten())
            else:
                # Simulation/Bypass: Just pass through or send dummy data
                # For line following, we might want to just pass the image to the next node
                pass
                
        except Exception as e:
            rospy.logerr(f"Inference callback error: {e}")

    def publish_results(self, data):
        msg = Float32MultiArray()
        msg.data = data.tolist()
        self.result_pub.publish(msg)

if __name__ == '__main__':
    try:
        node = NPUInferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
