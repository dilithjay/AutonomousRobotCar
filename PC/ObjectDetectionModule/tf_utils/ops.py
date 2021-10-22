# Copyright 2017 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""A module for helper tensorflow ops."""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import collections
import math
import six

from six.moves import range
from six.moves import zip
import tensorflow as tf
import tf_slim as slim


def expanded_shape(orig_shape, start_dim, num_dims):
    """Inserts multiple ones into a shape vector.

  Inserts an all-1 vector of length num_dims at position start_dim into a shape.
  Can be combined with tf.reshape to generalize tf.expand_dims.

  Args:
    orig_shape: the shape into which the all-1 vector is added (int32 vector)
    start_dim: insertion position (int scalar)
    num_dims: length of the inserted all-1 vector (int scalar)
  Returns:
    An int32 vector of length tf.size(orig_shape) + num_dims.
  """
    with tf.name_scope('ExpandedShape'):
        start_dim = tf.expand_dims(start_dim, 0)  # scalar to rank-1
        before = tf.slice(orig_shape, [0], start_dim)
        add_shape = tf.ones(tf.reshape(num_dims, [1]), dtype=tf.int32)
        after = tf.slice(orig_shape, start_dim, [-1])
        new_shape = tf.concat([before, add_shape, after], 0)
        return new_shape


def meshgrid(x, y):
    """Tiles the contents of x and y into a pair of grids.

  Multidimensional analog of numpy.meshgrid, giving the same behavior if x and y
  are vectors. Generally, this will give:

  xgrid(i1, ..., i_m, j_1, ..., j_n) = x(j_1, ..., j_n)
  ygrid(i1, ..., i_m, j_1, ..., j_n) = y(i_1, ..., i_m)

  Keep in mind that the order of the arguments and outputs is reverse relative
  to the order of the indices they go into, done for compatibility with numpy.
  The output tensors have the same shapes.  Specifically:

  xgrid.get_shape() = y.get_shape().concatenate(x.get_shape())
  ygrid.get_shape() = y.get_shape().concatenate(x.get_shape())

  Args:
    x: A tensor of arbitrary shape and rank. xgrid will contain these values
       varying in its last dimensions.
    y: A tensor of arbitrary shape and rank. ygrid will contain these values
       varying in its first dimensions.
  Returns:
    A tuple of tensors (xgrid, ygrid).
  """
    with tf.name_scope('Meshgrid'):
        x = tf.convert_to_tensor(x)
        y = tf.convert_to_tensor(y)
        x_exp_shape = expanded_shape(tf.shape(x), 0, tf.rank(y))
        y_exp_shape = expanded_shape(tf.shape(y), tf.rank(y), tf.rank(x))

        xgrid = tf.tile(tf.reshape(x, x_exp_shape), y_exp_shape)
        ygrid = tf.tile(tf.reshape(y, y_exp_shape), x_exp_shape)
        new_shape = y.get_shape().concatenate(x.get_shape())
        xgrid.set_shape(new_shape)
        ygrid.set_shape(new_shape)

        return xgrid, ygrid


def fixed_padding(inputs, kernel_size, rate=1):
    """Pads the input along the spatial dimensions independently of input size.

  Args:
    inputs: A tensor of size [batch, height_in, width_in, channels].
    kernel_size: The kernel to be used in the conv2d or max_pool2d operation.
                 Should be a positive integer.
    rate: An integer, rate for atrous convolution.

  Returns:
    output: A tensor of size [batch, height_out, width_out, channels] with the
      input, either intact (if kernel_size == 1) or padded (if kernel_size > 1).
  """
    kernel_size_effective = kernel_size + (kernel_size - 1) * (rate - 1)
    pad_total = kernel_size_effective - 1
    pad_beg = pad_total // 2
    pad_end = pad_total - pad_beg
    padded_inputs = tf.pad(inputs, [[0, 0], [pad_beg, pad_end],
                                    [pad_beg, pad_end], [0, 0]])
    return padded_inputs


def padded_one_hot_encoding(indices, depth, left_pad):
    """Returns a zero padded one-hot tensor.

  This function converts a sparse representation of indices (e.g., [4]) to a
  zero padded one-hot representation (e.g., [0, 0, 0, 0, 1] with depth = 4 and
  left_pad = 1). If `indices` is empty, the result will simply be a tensor of
  shape (0, depth + left_pad). If depth = 0, then this function just returns
  `None`.

  Args:
    indices: an integer tensor of shape [num_indices].
    depth: depth for the one-hot tensor (integer).
    left_pad: number of zeros to left pad the one-hot tensor with (integer).

  Returns:
    padded_onehot: a tensor with shape (num_indices, depth + left_pad). Returns
      `None` if the depth is zero.

  Raises:
    ValueError: if `indices` does not have rank 1 or if `left_pad` or `depth are
      either negative or non-integers.
  """
    if depth < 0 or not isinstance(depth, six.integer_types):
        raise ValueError('`depth` must be a non-negative integer.')
    if left_pad < 0 or not isinstance(left_pad, six.integer_types):
        raise ValueError('`left_pad` must be a non-negative integer.')
    if depth == 0:
        return None

    rank = len(indices.get_shape().as_list())
    if rank != 1:
        raise ValueError('`indices` must have rank 1, but has rank=%s' % rank)

    def one_hot_and_pad():
        one_hot = tf.cast(tf.one_hot(tf.cast(indices, tf.int64), depth,
                                     on_value=1, off_value=0), tf.float32)
        return tf.pad(one_hot, [[0, 0], [left_pad, 0]], mode='CONSTANT')

    result = tf.cond(tf.greater(tf.size(indices), 0), one_hot_and_pad,
                     lambda: tf.zeros((tf.size(indices), depth + left_pad)))
    return tf.reshape(result, [-1, depth + left_pad])


def dense_to_sparse_boxes(dense_locations, dense_num_boxes, num_classes):
    """Converts bounding boxes from dense to sparse form.

  Args:
    dense_locations:  a [max_num_boxes, 4] tensor in which only the first k rows
      are valid bounding box location coordinates, where k is the sum of
      elements in dense_num_boxes.
    dense_num_boxes: a [max_num_classes] tensor indicating the counts of
       various bounding box classes e.g. [1, 0, 0, 2] means that the first
       bounding box is of class 0 and the second and third bounding boxes are
       of class 3. The sum of elements in this tensor is the number of valid
       bounding boxes.
    num_classes: number of classes

  Returns:
    box_locations: a [num_boxes, 4] tensor containing only valid bounding
       boxes (i.e. the first num_boxes rows of dense_locations)
    box_classes: a [num_boxes] tensor containing the classes of each bounding
       box (e.g. dense_num_boxes = [1, 0, 0, 2] => box_classes = [0, 3, 3]
  """

    num_valid_boxes = tf.reduce_sum(dense_num_boxes)
    box_locations = tf.slice(dense_locations,
                             tf.constant([0, 0]), tf.stack([num_valid_boxes, 4]))
    tiled_classes = [tf.tile([i], tf.expand_dims(dense_num_boxes[i], 0))
                     for i in range(num_classes)]
    box_classes = tf.concat(tiled_classes, 0)
    box_locations.set_shape([None, 4])
    return box_locations, box_classes


def indices_to_dense_vector(indices,
                            size,
                            indices_value=1.,
                            default_value=0,
                            dtype=tf.float32):
    """Creates dense vector with indices set to specific value and rest to zeros.

  This function exists because it is unclear if it is safe to use
    tf.sparse_to_dense(indices, [size], 1, validate_indices=False)
  with indices which are not ordered.
  This function accepts a dynamic size (e.g. tf.shape(tensor)[0])

  Args:
    indices: 1d Tensor with integer indices which are to be set to
        indices_values.
    size: scalar with size (integer) of output Tensor.
    indices_value: values of elements specified by indices in the output vector
    default_value: values of other elements in the output vector.
    dtype: data type.

  Returns:
    dense 1D Tensor of shape [size] with indices set to indices_values and the
        rest set to default_value.
  """
    size = tf.cast(size, dtype=tf.int32)
    zeros = tf.ones([size], dtype=dtype) * default_value
    values = tf.ones_like(indices, dtype=dtype) * indices_value

    return tf.dynamic_stitch([tf.range(size), tf.cast(indices, dtype=tf.int32)],
                             [zeros, values])


def reduce_sum_trailing_dimensions(tensor, ndims):
    """Computes sum across all dimensions following first `ndims` dimensions."""
    return tf.reduce_sum(tensor, axis=tuple(range(ndims, tensor.shape.ndims)))


def normalize_to_target(inputs,
                        target_norm_value,
                        dim,
                        epsilon=1e-7,
                        trainable=True,
                        scope='NormalizeToTarget',
                        summarize=True):
    """L2 normalizes the inputs across the specified dimension to a target norm.

  This op implements the L2 Normalization layer introduced in
  Liu, Wei, et al. "SSD: Single Shot MultiBox Detector."
  and Liu, Wei, Andrew Rabinovich, and Alexander C. Berg.
  "Parsenet: Looking wider to see better." and is useful for bringing
  activations from multiple layers in a convnet to a standard scale.

  Note that the rank of `inputs` must be known and the dimension to which
  normalization is to be applied should be statically defined.

  Args:
    inputs: A `Tensor` of arbitrary size.
    target_norm_value: A float value that specifies an initial target norm or
      a list of floats (whose length must be equal to the depth along the
      dimension to be normalized) specifying a per-dimension multiplier
      after normalization.
    dim: The dimension along which the input is normalized.
    epsilon: A small value to add to the inputs to avoid dividing by zero.
    trainable: Whether the norm is trainable or not
    scope: Optional scope for variable_scope.
    summarize: Whether or not to add a tensorflow summary for the op.

  Returns:
    The input tensor normalized to the specified target norm.

  Raises:
    ValueError: If dim is smaller than the number of dimensions in 'inputs'.
    ValueError: If target_norm_value is not a float or a list of floats with
      length equal to the depth along the dimension to be normalized.
  """
    with tf.variable_scope(scope, 'NormalizeToTarget', [inputs]):
        if not inputs.get_shape():
            raise ValueError('The input rank must be known.')
        input_shape = inputs.get_shape().as_list()
        input_rank = len(input_shape)
        if dim < 0 or dim >= input_rank:
            raise ValueError(
                'dim must be non-negative but smaller than the input rank.')
        if not input_shape[dim]:
            raise ValueError('input shape should be statically defined along '
                             'the specified dimension.')
        depth = input_shape[dim]
        if not (isinstance(target_norm_value, float) or
                (isinstance(target_norm_value, list) and
                 len(target_norm_value) == depth) and
                all([isinstance(val, float) for val in target_norm_value])):
            raise ValueError('target_norm_value must be a float or a list of floats '
                             'with length equal to the depth along the dimension to '
                             'be normalized.')
        if isinstance(target_norm_value, float):
            initial_norm = depth * [target_norm_value]
        else:
            initial_norm = target_norm_value
        target_norm = slim.model_variable(
            name='weights',
            dtype=tf.float32,
            initializer=tf.constant(initial_norm, dtype=tf.float32),
            trainable=trainable)
        if summarize:
            mean = tf.reduce_mean(target_norm)
            tf.summary.scalar(tf.get_variable_scope().name, mean)
        lengths = epsilon + tf.sqrt(tf.reduce_sum(tf.square(inputs), dim, True))
        mult_shape = input_rank * [1]
        mult_shape[dim] = depth
        return tf.reshape(target_norm, mult_shape) * tf.truediv(inputs, lengths)


def reframe_image_corners_relative_to_boxes(boxes):
    """Reframe the image corners ([0, 0, 1, 1]) to be relative to boxes.

  The local coordinate frame of each box is assumed to be relative to
  its own for corners.

  Args:
    boxes: A float tensor of [num_boxes, 4] of (ymin, xmin, ymax, xmax)
      coordinates in relative coordinate space of each bounding box.

  Returns:
    reframed_boxes: Reframes boxes with same shape as input.
  """
    ymin, xmin, ymax, xmax = tf.unstack(boxes, axis=1)

    height = tf.maximum(ymax - ymin, 1e-4)
    width = tf.maximum(xmax - xmin, 1e-4)

    ymin_out = (0 - ymin) / height
    xmin_out = (0 - xmin) / width
    ymax_out = (1 - ymin) / height
    xmax_out = (1 - xmin) / width
    return tf.stack([ymin_out, xmin_out, ymax_out, xmax_out], axis=1)


def reframe_box_masks_to_image_masks(box_masks, boxes, image_height,
                                     image_width, resize_method='bilinear'):
    """Transforms the box masks back to full image masks.

  Embeds masks in bounding boxes of larger masks whose shapes correspond to
  image shape.

  Args:
    box_masks: A tensor of size [num_masks, mask_height, mask_width].
    boxes: A tf.float32 tensor of size [num_masks, 4] containing the box
           corners. Row i contains [ymin, xmin, ymax, xmax] of the box
           corresponding to mask i. Note that the box corners are in
           normalized coordinates.
    image_height: Image height. The output mask will have the same height as
                  the image height.
    image_width: Image width. The output mask will have the same width as the
                 image width.
    resize_method: The resize method, either 'bilinear' or 'nearest'. Note that
      'bilinear' is only respected if box_masks is a float.

  Returns:
    A tensor of size [num_masks, image_height, image_width] with the same dtype
    as `box_masks`.
  """
    resize_method = 'nearest' if box_masks.dtype == tf.uint8 else resize_method

    def reframe_box_masks_to_image_masks_default():
        """The default function when there are more than 0 box masks."""

        num_boxes = tf.shape(box_masks)[0]
        box_masks_expanded = tf.expand_dims(box_masks, axis=3)

        resized_crops = tf.image.crop_and_resize(
            image=box_masks_expanded,
            boxes=reframe_image_corners_relative_to_boxes(boxes),
            box_ind=tf.range(num_boxes),
            crop_size=[image_height, image_width],
            method=resize_method,
            extrapolation_value=0)
        return tf.cast(resized_crops, box_masks.dtype)

    image_masks = tf.cond(
        tf.shape(box_masks)[0] > 0,
        reframe_box_masks_to_image_masks_default,
        lambda: tf.zeros([0, image_height, image_width, 1], box_masks.dtype))
    return tf.squeeze(image_masks, axis=3)


def fpn_feature_levels(num_levels, unit_scale_index, image_ratio, boxes):
    """Returns fpn feature level for each box based on its area.

  See section 4.2 of https://arxiv.org/pdf/1612.03144.pdf for details.

  Args:
    num_levels: An integer indicating the number of feature levels to crop boxes
      from.
    unit_scale_index: An 0-based integer indicating the index of feature map
      which most closely matches the resolution of the pretrained model.
    image_ratio: A float indicating the ratio of input image area to pretraining
      image area.
    boxes: A float tensor of shape [batch, num_boxes, 4] containing boxes of the
      form [ymin, xmin, ymax, xmax] in normalized coordinates.

  Returns:
    An int32 tensor of shape [batch_size, num_boxes] containing feature indices.
  """
    assert num_levels > 0, (
        '`num_levels` must be > 0. Found {}'.format(num_levels))
    assert unit_scale_index < num_levels and unit_scale_index >= 0, (
        '`unit_scale_index` must be in [0, {}). Found {}.'.format(
            num_levels, unit_scale_index))
    box_height_width = boxes[:, :, 2:4] - boxes[:, :, 0:2]
    areas_sqrt = tf.sqrt(tf.reduce_prod(box_height_width, axis=2))
    log_2 = tf.cast(tf.log(2.0), dtype=boxes.dtype)
    levels = tf.cast(
        tf.floordiv(tf.log(areas_sqrt * image_ratio), log_2)
        +
        unit_scale_index,
        dtype=tf.int32)
    levels = tf.maximum(0, tf.minimum(num_levels - 1, levels))
    return levels


def bfloat16_to_float32_nested(input_nested):
    """Convert float32 tensors in a nested structure to bfloat16.

  Args:
    input_nested: A Python dict, values being Tensor or Python list/tuple of
      Tensor or Non-Tensor.

  Returns:
    A Python dict with the same structure as `tensor_dict`,
    with all bfloat16 tensors converted to float32.
  """
    if isinstance(input_nested, tf.Tensor):
        if input_nested.dtype == tf.bfloat16:
            return tf.cast(input_nested, dtype=tf.float32)
        else:
            return input_nested
    elif isinstance(input_nested, (list, tuple)):
        out_tensor_dict = [bfloat16_to_float32_nested(t) for t in input_nested]
    elif isinstance(input_nested, dict):
        out_tensor_dict = {
            k: bfloat16_to_float32_nested(v) for k, v in input_nested.items()
        }
    else:
        return input_nested
    return out_tensor_dict


def gather_with_padding_values(input_tensor, indices, padding_value):
    """Gathers elements from tensor and pads `padding_value` for ignore indices.

  Gathers elements from `input_tensor` based on `indices`. If there are ignore
  indices (which are "-1"s) in `indices`, `padding_value` will be gathered for
  those positions.

  Args:
    input_tensor: A N-D tensor of shape [M, d_1, d_2 .. d_(N-1)] to gather
      values from.
    indices: A 1-D tensor in which each element is either an index in the
      first dimension of input_tensor or -1.
    padding_value: A (N-1)-D tensor of shape [d_1, d_2 .. d_(N-1)] which will be
      used as gathered value for each ignore index in `indices`.

  Returns:
    gathered_tensor: A tensor of shape [L, d_1, d_2 .. d_(N-1)] containing
      values gathered from input_tensor. The first dimension L is equal to the
      length of `indices`.
  """
    padding_value = tf.expand_dims(padding_value, axis=0)
    input_tensor = tf.concat([padding_value, input_tensor], axis=0)
    gather_indices = indices + 1
    gathered_tensor = tf.gather(input_tensor, gather_indices)
    return gathered_tensor


EqualizationLossConfig = collections.namedtuple('EqualizationLossConfig',
                                                ['weight', 'exclude_prefixes'])


def giou(boxes1, boxes2):
    """Computes generalized IOU between two tensors.

  Each box should be represented as [ymin, xmin, ymax, xmax].

  Args:
    boxes1: a tensor with shape [num_boxes, 4]
    boxes2: a tensor with shape [num_boxes, 4]

  Returns:
    a tensor of shape [num_boxes] containing GIoUs

  """
    pred_ymin, pred_xmin, pred_ymax, pred_xmax = tf.unstack(boxes1, axis=1)
    gt_ymin, gt_xmin, gt_ymax, gt_xmax = tf.unstack(boxes2, axis=1)

    gt_area = (gt_ymax - gt_ymin) * (gt_xmax - gt_xmin)
    pred_area = (pred_ymax - pred_ymin) * (pred_xmax - pred_xmin)

    x1_i = tf.maximum(pred_xmin, gt_xmin)
    x2_i = tf.minimum(pred_xmax, gt_xmax)
    y1_i = tf.maximum(pred_ymin, gt_ymin)
    y2_i = tf.minimum(pred_ymax, gt_ymax)
    intersection_area = tf.maximum(0.0, y2_i - y1_i) * tf.maximum(0.0,
                                                                  x2_i - x1_i)

    x1_c = tf.minimum(pred_xmin, gt_xmin)
    x2_c = tf.maximum(pred_xmax, gt_xmax)
    y1_c = tf.minimum(pred_ymin, gt_ymin)
    y2_c = tf.maximum(pred_ymax, gt_ymax)
    hull_area = (y2_c - y1_c) * (x2_c - x1_c)

    union_area = gt_area + pred_area - intersection_area
    iou = tf.where(tf.equal(union_area, 0.0),
                   tf.zeros_like(union_area), intersection_area / union_area)
    giou_ = iou - tf.where(hull_area > 0.0,
                           (hull_area - union_area) / hull_area, iou)
    return giou_


def center_to_corner_coordinate(input_tensor):
    """Converts input boxes from center to corner representation."""
    reshaped_encodings = tf.reshape(input_tensor, [-1, 4])
    ycenter = tf.gather(reshaped_encodings, [0], axis=1)
    xcenter = tf.gather(reshaped_encodings, [1], axis=1)
    h = tf.gather(reshaped_encodings, [2], axis=1)
    w = tf.gather(reshaped_encodings, [3], axis=1)
    ymin = ycenter - h / 2.
    xmin = xcenter - w / 2.
    ymax = ycenter + h / 2.
    xmax = xcenter + w / 2.
    return tf.squeeze(tf.stack([ymin, xmin, ymax, xmax], axis=1))
