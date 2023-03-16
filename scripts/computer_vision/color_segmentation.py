import math

import cv2
import numpy as np
import pdb

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
	# cv2.destroyAllWindows()

counter = 0

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
	global counter
	########## YOUR CODE STARTS HERE ##########
	# lower_orange = np.array([5, 220, 220])
	# upper_orange = np.array([20, 255, 255])
	# lower_orange = np.array([5, 220, 220])
	# upper_orange = np.array([50, 255, 255])
	# lower_orange = np.array([5, 220, 220])
	# upper_orange = np.array([75, 255, 255])
	# lower_orange = np.array([5, 220, 170])
	# upper_orange = np.array([50, 255, 255])
	# dilation
	# lower_orange = np.array([5, 235, 235])
	# upper_orange = np.array([20, 255, 255])
	# dilation on img
	lower_orange = np.array([5, 220, 180])
	upper_orange = np.array([30, 255, 255])

	# NOTE: dilation on hsv
	# hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	# print("HSV_IMAGE Shape: " + str(hsv_img.shape))
	# print("ORIGINAL_IMAGE Shape: " + str(img.shape))
	# kernel = np.ones((2, 2), np.uint8)
	# img_dilation = cv2.dilate(hsv_img, kernel, iterations=1)

	# NOTE: dilation on img
	kernel = np.ones((3, 3), np.uint8)
	img_dilation = cv2.dilate(img, kernel, iterations=2)
	hsv_img = cv2.cvtColor(img_dilation, cv2.COLOR_BGR2HSV)

	# NOTE: erosion + dilation on hsv
	# hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	# kernel = np.ones((4, 4), np.uint8)
	# img_erosion = cv2.erode(hsv_img, kernel, iterations=2)
	# img_dilation = cv2.dilate(img_erosion, kernel, iterations=3)

	mask = cv2.inRange(hsv_img, lower_orange, upper_orange)
	# mask = cv2.inRange(img_dilation, lower_orange, upper_orange)
	# mask = cv2.inRange(hsv_img, lower_orange, upper_orange)

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

	# TODO: Remove testing code
	image_print(img)
	image_print(hsv_img)
	# image_print(img_erosion)
	image_print(img_dilation)
	image_print(result)
	result[lowest_x][lowest_y] = [255, 255, 255] # Top Left
	result[highest_x][highest_y] = [255, 255, 255] # Bottom Right
	matrix_bounding_box = ((lowest_x, lowest_y), (highest_x, highest_y))
	bounding_box = ((lowest_y, lowest_x), (highest_y, highest_x))
	# print("Dimensions: " + +)
	print("Matrix Bounding Box counter=" + str(counter) + ", bounding_box=" + str(matrix_bounding_box))
	print("Answer Bounding Box counter=" + str(counter) + ", bounding_box=" + str(bounding_box))
	# print("HSV_IMAGE Shape: " + str(hsv_img.shape))
	# print("RESULT_IMAGE Shape: " + str(result.shape))
	image_print(result)
	pixel_x = 0
	pixel_y = 0
	while pixel_x != -1:
		print(hsv_img[pixel_x][pixel_y])
		prev_val = result[pixel_x][pixel_y]
		prev_x = pixel_x
		prev_y = pixel_y
		result[pixel_x][pixel_y] = [0, 0, 255]
		# cv2.destroyAllWindows()
		# cv2.imshow("image", result)
		image_print(result)
		print("Input x pixel (-1 to quit):")
		pixel_x = int(input())
		print("Input y pixel (-1 to quit):")
		pixel_y = int(input())
		cv2.destroyAllWindows()
		result[prev_x][prev_y] = prev_val
	cv2.destroyAllWindows()
	counter += 1
	# bounding_box = ((highest_x, highest_y), (lowest_x, lowest_y))

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box


# TODO: Remove this after testing
# if __name__ == "__main__":
	# # cone = "./test_images_cone/cone_template.png"
	# # cone_img = cv2.imread(cone)
	# # print("cone image shape: " + str(cone_img.shape))
	# # hsv_cone_img = cv2.cvtColor(cone_img, cv2.COLOR_BGR2HSV)
	# # print("hsv cone image shape: " + str(hsv_cone_img.shape))
	# # print(hsv_cone_img)
	# # for i, row in enumerate(hsv_cone_img):
	# # 	print(row)
	# 	# # get the pixel values by iterating
	# 	# for j, pixel in enumerate(img):
	# 	# 	if (i == j or i + j == img.shape[0]):
	# 	# 		# update the pixel value to black
	# 	# 		img[i][j] = [0, 0, 0]
	# # image_print(hsv_cone_img)
	# test1_cropped = "./test_images_cone/test1_cropped.jpg"
	# cropped_img = cv2.imread(test1_cropped)
	# image_print(cropped_img)
	# print("image shape: " + str(cropped_img.shape))
	# hsv_cone_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
	# print("hsv cone image shape: " + str(hsv_cone_img.shape))
	# # print(hsv_cone_img)
	# # for i, row in enumerate(hsv_cone_img):
	# # 	print(row)
	# # # get the pixel values by iterating
	# # for j, pixel in enumerate(img):
	# # 	if (i == j or i + j == img.shape[0]):
	# # 		# update the pixel value to black
	# # 		img[i][j] = [0, 0, 0]
	# image_print(hsv_cone_img)
	# # It converts the BGR color space of image to HSV color space
	# # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	#
	# # Threshold of blue in HSV space
	# lower_orange = np.array([5, 220, 220])
	# upper_orange = np.array([20, 255, 255])
	#
	# # preparing the mask to overlay
	# mask = cv2.inRange(hsv_cone_img, lower_orange, upper_orange)
	#
	# # The black region in the mask has the value of 0,
	# # so when multiplied with original image removes all non-blue regions
	# result = cv2.bitwise_and(cropped_img, cropped_img, mask=mask)
	# image_print(result)
	# lowest_x = math.inf
	# lowest_y = math.inf
	# highest_x = -math.inf
	# highest_y = -math.inf
	# for i, row in enumerate(result):
	# 	# print(row)
	# 	# get the pixel values by iterating
	# 	for j, pixel in enumerate(result[0]):
	# 		# if (i == j or i + j == result.shape[0]):
	# 		if result[i][j][1] > 0:
	# 			# find non masked pixels
	# 			if i < lowest_x:
	# 				lowest_x = i
	#
	# 			if j < lowest_y:
	# 				lowest_y = j
	#
	# 			if i > highest_x:
	# 				highest_x = i
	#
	# 			if j > highest_y:
	# 				highest_y = j
	# 			# update the pixel value to black
	# 			# result[i][j] = [0, 0, 0]
	#
	# print("Lowest corner: (" + str(lowest_x) + ", " + str(lowest_y) + ")")
	# print("Highest corner: (" + str(highest_x) + ", " + str(highest_y) + ")")
	# result[lowest_x][lowest_y] = [255, 255, 255] # Top Left
	# result[highest_x][highest_y] = [255, 255, 255] # Bottom Right
	# image_print(result)
	# # test1 = "./test_images_cone/test1.jpg"
	# # img = cv2.imread(test1)
	# # print("image shape: " + str(img.shape))
	# # image_print(img)
	# # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	# # image_print(hsv_img)
	# cv2.destroyAllWindows()

