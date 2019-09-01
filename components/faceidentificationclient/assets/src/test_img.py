import sys
import argparse
import time

import tensorflow as tf
import cv2
import numpy as np
import os

from mtcnn import PNet, RNet, ONet
from tools import detect_face, get_model_filenames
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 


def bounding_box_face(img, gpu_mem):

	### Model parameters
	model_dir = 'assets/save_model/'
	minsize = 20
	# factor = 0.7
	# threshold = [0.8, 0.8, 0.8]
	factor = 0.709
	threshold = [0.6, 0.7, 0.7]

	file_paths = get_model_filenames(model_dir)
	with tf.device('/gpu:0'):
		with tf.Graph().as_default():
			config = tf.ConfigProto(allow_soft_placement=True)
			config.gpu_options.per_process_gpu_memory_fraction = gpu_mem
			with tf.Session(config=config) as sess:
				if len(file_paths) == 3:
					image_pnet = tf.placeholder(
						tf.float32, [None, None, None, 3])
					pnet = PNet({'data': image_pnet}, mode='test')
					out_tensor_pnet = pnet.get_all_output()

					image_rnet = tf.placeholder(tf.float32, [None, 24, 24, 3])
					rnet = RNet({'data': image_rnet}, mode='test')
					out_tensor_rnet = rnet.get_all_output()

					image_onet = tf.placeholder(tf.float32, [None, 48, 48, 3])
					onet = ONet({'data': image_onet}, mode='test')
					out_tensor_onet = onet.get_all_output()

					saver_pnet = tf.train.Saver(
									[v for v in tf.global_variables()
									 if v.name[0:5] == "pnet/"])
					saver_rnet = tf.train.Saver(
									[v for v in tf.global_variables()
									 if v.name[0:5] == "rnet/"])
					saver_onet = tf.train.Saver(
									[v for v in tf.global_variables()
									 if v.name[0:5] == "onet/"])

					saver_pnet.restore(sess, file_paths[0])

					def pnet_fun(img): return sess.run(
						out_tensor_pnet, feed_dict={image_pnet: img})

					saver_rnet.restore(sess, file_paths[1])

					def rnet_fun(img): return sess.run(
						out_tensor_rnet, feed_dict={image_rnet: img})

					saver_onet.restore(sess, file_paths[2])

					def onet_fun(img): return sess.run(
						out_tensor_onet, feed_dict={image_onet: img})

				else:
					saver = tf.train.import_meta_graph(file_paths[0])
					saver.restore(sess, file_paths[1])

					def pnet_fun(img): return sess.run(
						('softmax/Reshape_1:0',
						 'pnet/conv4-2/BiasAdd:0'),
						feed_dict={
							'Placeholder:0': img})

					def rnet_fun(img): return sess.run(
						('softmax_1/softmax:0',
						 'rnet/conv5-2/rnet/conv5-2:0'),
						feed_dict={
							'Placeholder_1:0': img})

					def onet_fun(img): return sess.run(
						('softmax_2/softmax:0',
						 'onet/conv6-2/onet/conv6-2:0',
						 'onet/conv6-3/onet/conv6-3:0'),
						feed_dict={
							'Placeholder_2:0': img})

				rectangles, points = detect_face(img, minsize,
												 pnet_fun, rnet_fun, onet_fun,
												 threshold, factor)

	tf.reset_default_graph()
	return rectangles, points
