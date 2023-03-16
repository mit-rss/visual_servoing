import math

import cv2
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  Y  > > > > >
#
#  X
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


def cd_color_segmentation(img, template):
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
	# NOTE: cone color boundaries
	lower_orange = np.array([5, 220, 180])
	upper_orange = np.array([30, 255, 255])

	# NOTE: dilation on img
	kernel = np.ones((3, 3), np.uint8)
	img_dilation = cv2.dilate(img, kernel, iterations=2)
	hsv_img = cv2.cvtColor(img_dilation, cv2.COLOR_BGR2HSV)

	mask = cv2.inRange(hsv_img, lower_orange, upper_orange)

	result = cv2.bitwise_and(img, img, mask=mask)
	lowest_x = float('inf')
	lowest_y = float('inf')
	highest_x = -float('inf')
	highest_y = -float('inf')
	for i, row in enumerate(result):
		# print(row)
		# get the pixel values by iterating
		for j, pixel in enumerate(result[0]):
			# if (i == j or i + j == result.shape[0]):
			if result[i][j][1] > 0:
				# find non masked pixels
				if i < lowest_x:
					lowest_x = i

				if j < lowest_y:
					lowest_y = j

				if i > highest_x:
					highest_x = i

				if j > highest_y:
					highest_y = j

	# NOTE: Due to image coordinate frames, conventional dimensions got swapped along the way
	bounding_box = ((lowest_y, lowest_x), (highest_y, highest_x))
	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box
