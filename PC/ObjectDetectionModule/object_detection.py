from PC.ObjectDetectionModule.od_handlers import \
    ODHandlerType, PedestrianODHandler, VehicleODHandler, TrafficLightODHandler

import tensorflow as tf
import numpy as np

from PC.ObjectDetectionModule.tf_utils import ops as utils_ops
from PC.ObjectDetectionModule.tf_utils import label_map_util
from PC.ObjectDetectionModule.tf_utils import visualization_utils as vis_util


class ObjectDetection:
    def __init__(self, handler_types: set, threshold=0.5):
        self.handlers = {}
        for handler in handler_types:
            if handler == ODHandlerType.PEDESTRIAN:
                self.handlers[handler] = PedestrianODHandler()
            elif handler == ODHandlerType.VEHICLE:
                self.handlers[handler] = VehicleODHandler()
            else:   # handler == ODHandlerType.TRAFFIC_LIGHT:
                self.handlers[handler] = TrafficLightODHandler()

        self.threshold = threshold
        model_dir = 'ObjectDetectionModule/models/my_model/saved_model'
        self.model = tf.saved_model.load(str(model_dir))
        self.class_index = label_map_util.create_category_index_from_labelmap('ObjectDetectionModule/label_map.pbtxt',
                                                                              use_display_name=True)

    def run_inference(self, image):
        image = np.asarray(image)
        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image)
        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis, ...]

        # Run inference
        model_fn = self.model.signatures['serving_default']
        output_dict = model_fn(input_tensor)

        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(output_dict.pop('num_detections'))
        output_dict = {key: value[0, :num_detections].numpy()
                       for key, value in output_dict.items()}
        output_dict['num_detections'] = num_detections

        # detection_classes should be ints.
        output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)

        # Handle models with masks:
        if 'detection_masks' in output_dict:
            # Reframe the the bbox mask to the image size.
            detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                output_dict['detection_masks'], output_dict['detection_boxes'],
                image.shape[0], image.shape[1])
            detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                               tf.uint8)
            output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()

        return output_dict

    def get_speed_multiplier(self, image, show_det=False):
        output_dict = self.run_inference(image)
        min_multiplier = 1
        for handler in self.handlers:
            min_multiplier = min(min_multiplier, self.handlers[handler].
                                 get_speed_multiplier(self.get_filtered_output_dict(output_dict)))

        if show_det:
            vis_util.visualize_boxes_and_labels_on_image_array(
                image,
                output_dict['detection_boxes'],
                output_dict['detection_classes'],
                output_dict['detection_scores'],
                self.class_index,
                instance_masks=output_dict.get('detection_masks_reframed', None),
                use_normalized_coordinates=True,
                line_thickness=8)

        return min_multiplier, image

    def get_filtered_output_dict(self, output_dict):
        num_detections = output_dict['num_detections']

        new_dict = {}
        for index in self.class_index:
            new_dict[self.class_index[index]['name']] = []

        for i in range(num_detections):
            if output_dict['detection_scores'][i] > self.threshold:
                class_name = self.class_index[output_dict['detection_classes'][i]]['name']
                print(class_name)
                new_dict[class_name].append(output_dict['detection_boxes'][i])
        return new_dict
