import cv2
import numpy as np
import pdb
import matplotlib
matplotlib.use('Agg')
import os

import matplotlib.pyplot as plt

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template=None, img_path=None):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	#img = cv2.imread("test_images_cone/test1.jpg")
	img_name = img_path.split("/")[-1]
	img_bb_name = os.path.join("test_images_cone_bb", img_name)


	bounding_box = ((0,0),(0,0))
	img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)

	light_orange = (1, 190, 200)
	dark_orange = (25, 255, 255)

	mask = cv2.inRange(img_hsv, light_orange, dark_orange)
	result = cv2.bitwise_and(img_rgb, img_rgb, mask=mask)

	result= cv2.GaussianBlur(result, (7,7), 0) #helps with getting bottom edges of the cone

	## IN PROGRESS: TRYING TO WORK WITH CONTOURS ##
	# color_cone_x, color_cone_y, _ = np.nonzero(result)
	# color_bounding_box = ((np.min(color_coney), np.min(color_cone_x)), (np.max(color_cone_y), np.max(color_cone_x))) #np.max(cone_y)))

	# img_bb = cv2.rectangle(img_rgb, bounding_box[0], bounding_box[1], (255,0,0), 2)

	# im_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	# ret, thresh = cv2.threshold(im_gray, 127, 255, 0)
	# contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


	color_cone_x, color_cone_y, _ = np.nonzero(result)
	bounding_box = ((np.min(color_cone_y), np.min(color_cone_x)), (np.max(color_cone_y), np.max(color_cone_x)))
	img_bb = cv2.rectangle(img_rgb, bounding_box[0], bounding_box[1], (255,0,0), 2)


	plt.imshow(img_bb) #, bounding_box[0], bounding_box[1])
	plt.savefig(img_bb_name)
	plt.show(result)

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box
