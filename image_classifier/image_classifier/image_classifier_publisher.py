import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import ObjectHypothesis
from vision_msgs.msg import Classification
from edge_impulse_linux.image import ImageImpulseRunner
import cv2
import os

class ImageClassifierPub(Node):

    def __init__(self):

        self.occupied = False
        self.img = None
        self.cv_bridge = CvBridge()

        super().__init__('image_classifier_pub')
        self.init_parameters()
        self.ei_classifier = self.EI_Classifier(self.modelfile, self.get_logger())

        self.image_publisher = self.create_publisher(Image,'/classification/output/image',1)
        self.results_publisher = self.create_publisher(Classification,'/classification/output/results',1)

        self.timer_classify = self.create_timer(2,self.classify_callback)
        self.timer_classify.cancel()
        self.subscription = self.create_subscription(Image,'/classification/input/image',self.listener_callback,1)
        self.subscription

    def init_parameters(self):
        self.declare_parameter('model.filepath',os.path.expanduser('~') + '/ros2_ws/src/image_classifier/resource/modelfile.eim')
        self.modelfile= self.get_parameter('model.filepath').get_parameter_value().string_value

        self.declare_parameter('frame_id','base_link')
        self.frame_id= self.get_parameter('frame_id').get_parameter_value().string_value

        self.declare_parameter('show.labels',True)
        self.show_labels_on_image = self.get_parameter('show.labels').get_parameter_value().bool_value

        self.declare_parameter('show.classification_info', False)
        self.show_extra_classification_info = self.get_parameter('show.classification_info').get_parameter_value().bool_value

        self.declare_parameter('use_webcam', False)
        self.use_webcam_image = self.get_parameter('use_webcam').get_parameter_value().bool_value

    def listener_callback(self, msg):
        if not self.use_webcam_image:
            if len(msg.data):
                current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
                img_encoding = msg.encoding
                if img_encoding=="bgr8":
                    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
                if not self.occupied:
                    self.img = current_frame
                    self.timer_classify.reset()  

    def classify_callback(self):
        self.occupied = True

        # vision msgs
        results_msg = Classification()
        time_now = self.get_clock().now().to_msg()
        results_msg.header.stamp = time_now
        results_msg.header.frame_id = self.frame_id

        if self.use_webcam_image:
            self.cap = cv2.VideoCapture(0)
            ret, frame = self.cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.img = frame
            self.cap.release()
        
        # classify
        features, cropped, res = self.ei_classifier.classify(self.img)

        #prepare output
        if "classification" in res["result"].keys():
            if self.show_extra_classification_info:
                self.get_logger().info('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
            for label in self.ei_classifier.labels:
                score = res['result']['classification'][label]
                result = ObjectHypothesis()
                result.class_id = label
                result.score = score
                results_msg.results.append(result)
                if self.show_extra_classification_info:
                    self.get_logger().info('%s: %.2f\t' % (label, score), end='')

                if self.show_labels_on_image:
                    # append label into 'cropped' image data
                    pass

        cropped=cv2.cvtColor(cropped, cv2.COLOR_RGB2BGR)

        # publish message
        self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(cropped,"bgr8"))
        self.results_publisher.publish(results_msg)

        self.occupied= False
        self.timer_classify.reset()
        
    
    class EI_Classifier:
        def __init__(self, modelfile, logger):
            self.runner = None
            self.labels = None
            self.logger = logger
            with ImageImpulseRunner(modelfile) as self.runner:
                try:
                    self.model_info = self.runner.init()
                    #print(self.model_info)
                    self.logger.info('Model loaded successfully!')
                    self.logger.info('Model owner: '+ self.model_info['project']['owner'])
                    self.logger.info('Model name: ' + self.model_info['project']['name'])
                    self.logger.info('Model version: ' + str(self.model_info['project']['deploy_version']))
                    self.logger.info('Model type: '+ self.model_info['model_parameters']['model_type'])
                    self.labels = self.model_info['model_parameters']['labels']
                    
                except:
                    self.logger.error('Issue on opening .eim file')
                    if (self.runner):
                        self.runner.stop()

        def __del__(self):
            if (self.runner):
                self.runner.stop()  

        def classify(self, img):
            try:
                # classification
                features, cropped = self.runner.get_features_from_image(img)
                res = self.runner.classify(features)
                return features, cropped, res
            except:
                self.logger.error('Error in classification')


def main():
    rclpy.init()
    image_classifier_publisher_node = ImageClassifierPub()

    if image_classifier_publisher_node.use_webcam_image:
        image_classifier_publisher_node.timer_classify.reset()

    rclpy.spin(image_classifier_publisher_node)

    image_classifier_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


