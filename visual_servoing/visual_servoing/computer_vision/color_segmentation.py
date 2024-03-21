import cv2
import numpy as np
# import imutils

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

def create_trackbars():
    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255
        for j in "HSV":
            cv2.createTrackbar("%s_%s" % (j, i), "trackbars", v, 255, callback)

def get_trackbar_values():
    values = []
    for i in ["MIN", "MAX"]:
        for j in "HSV":
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "trackbars")
            values.append(v)
    return values

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
	cv2.namedWindow("original", cv2.WINDOW_NORMAL)
	cv2.namedWindow("hsv", cv2.WINDOW_NORMAL)
	cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
	# cv2.namedWindow("trackbars", 0)
	# create_trackbars()
	# while True:
          
	H_min ,S_min ,V_min ,H_max ,S_max ,V_max = [5,203,130,30,255,255]#get_trackbar_values()#
	print(f'h {H_min},s {S_min},v {V_min},h {H_max},s{S_max},v {V_max}')
	frame_to_mask = cv2.cvtColor(img , cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(frame_to_mask , (H_min , S_min , V_min), (H_max , S_max , V_max))
# want to remove everything except a bar in the middle
    # robot frame (1, 0) is (350, 244)
	mask_height, mask_width = mask.shape[:2]
	visible_bar_height = 10
	mask = cv2.rectangle(mask, (0, 217), (mask_width-1, mask_height-1), (0,0,0), -1)
	mask = cv2.rectangle(mask, (0, 0), (mask_width-1, 244-visible_bar_height), (0,0,0), -1)
		
	contr, heir = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	c = max(contr, key = cv2.contourArea)
	x, y, w, h = cv2.boundingRect(c)
	cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
	

		
	cv2.imshow("original", img)
	cv2.imshow("hsv", frame_to_mask)
	cv2.imshow("mask", mask)
	cv2.waitKey(0)

	# blurred = cv2.GaussianBlur(img , (11,11), 0)
	# maskErode = cv2.erode(mask , None , iterations=2)
	# maskDilate = cv2.dilate(mask , None , iterations=2)
	# contr, heir = cv2.findContours(maskDilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	# print(f'first contr is {len(contr[0])}')
	# print(contr[0])
	# x, y, w, h = cv2.boundingRect(contr[0])
	# cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
     
	# flags = [i for i in dir(cv2) if i.startswith('COLOR_')]
	# print( flags )
     
	# cv2.imshow("original", img)
	# cv2.imshow("hsv", frame_to_mask)
	# cv2.imshow("mask", mask)
	# cv2.waitKey(0)
	
	# contr = imutils.grab_contours(contr)
	bounding_box = ((x,y),(x+w,y+h))

	print(bounding_box)

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box

def callback(value):
    pass





