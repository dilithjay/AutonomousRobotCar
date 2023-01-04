from PC.ObjectDetectionModule.od_handlers import \
    ODHandlerType, PedestrianODHandler, VehicleODHandler, TrafficLightODHandler

import tensorflow as tf
import numpy as np

from PC.ObjectDetectionModule.tf_utils import ops as utils_ops
from PC.ObjectDetectionModule.tf_utils import label_map_util
from PC.ObjectDetectionModule.tf_utils import visualization_utils as vis_util


class ObjectDetection:
    def __init__(self):
        self.handlers = {
            ODHandlerType.PEDESTRIAN: PedestrianODHandler(),
            ODHandlerType.VEHICLE: VehicleODHandler(),
            ODHandlerType.TRAFFIC_LIGHT: TrafficLightODHandler()
        }

        model_dir = 'ObjectDetectionModule/models/my_model_5/saved_model'
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
        output_dict, filtered_dict = self.get_filtered_output_dict(output_dict)

        for handler in self.handlers:
            min_multiplier = min(min_multiplier, self.handlers[handler].
                                 get_speed_multiplier(filtered_dict))

        if show_det:
            vis_util.visualize_boxes_and_labels_on_image_array(
                image,
                output_dict['detection_boxes'],
                output_dict['detection_classes'],
                output_dict['detection_scores'],
                self.class_index,
                instance_masks=output_dict.get('detection_masks_reframed', None),
                use_normalized_coordinates=True,
                line_thickness=8,
                min_score_thresh=.2)

        return min_multiplier, image

    def get_filtered_output_dict(self, output_dict):
        num_detections = output_dict['num_detections']
        per_class_threshold = {"vehicle": 0.99, "pedestrian": 0.99, "light_red": 0.2, "light_green": 0.2}

        new_dict = {'detection_boxes': [], 'detection_classes': [], 'detection_scores': []}
        filtered_dict = {}
        for index in self.class_index:
            filtered_dict[self.class_index[index]['name']] = []

        for i in range(num_detections):
            class_name = self.class_index[output_dict['detection_classes'][i]]['name']
            if output_dict['detection_scores'][i] > per_class_threshold[class_name]:
                for key in new_dict:
                    new_dict[key].append(output_dict[key][i])
                y1, x1, y2, x2 = output_dict['detection_boxes'][i]
                filtered_dict[class_name].append(((x1 + x2) / 2, (y1 + y2) / 2))

        for key in new_dict:
            new_dict[key] = np.array(new_dict[key])

        return new_dict, filtered_dict

    def get_filtered_output_dict1(self, output_dict):
        num_detections = output_dict['num_detections']
        per_class_threshold = {"vehicle": 0.99, "pedestrian": 0.99, "light_red": 0.3, "light_green": 0.4}

        new_dict = {}
        for index in self.class_index:
            new_dict[self.class_index[index]['name']] = []

        for i in range(num_detections):
            class_name = self.class_index[output_dict['detection_classes'][i]]['name']
            if output_dict['detection_scores'][i] > per_class_threshold[class_name]:
                y1, x1, y2, x2 = output_dict['detection_boxes'][i]
                new_dict[class_name].append(((x1 + x2) / 2, (y1 + y2) / 2))

        return new_dict
