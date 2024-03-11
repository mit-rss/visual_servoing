import cv2
import numpy as np
import csv
import ast
import sys
from sift_template import cd_sift_ransac, cd_template_matching
from color_segmentation import cd_color_segmentation

# File paths
cone_csv_path = "./test_images_cone/test_images_cone.csv"
citgo_csv_path = "./test_images_citgo/test_citgo.csv"
localization_csv_path="./test_images_localization/test_localization.csv"

cone_template_path = './test_images_cone/cone_template.png'
citgo_template_path = './test_images_citgo/citgo_template.png'
localization_template_path='./test_images_localization/basement_fixed.png'

cone_score_path = './scores/test_scores_cone.csv'
citgo_score_path = './scores/test_scores_citgo.csv'
localization_score_path = './scores/test_scores_map.csv'

def iou_score(bbox1, bbox2):
    """
    Return the IoU score for the two bounding boxes
    Input:
        bbox1: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
        bbox2: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
    Return:
        score: float; the IoU score
    """
    # First check bbox is coming in the correct order
    if bbox1[0][0] > bbox1[1][0] or bbox1[0][1] > bbox1[1][1]:
        print ("Check that you are returning bboxes as ((xmin, ymin),(xmax,ymax))")
    # Determine intersection rectangle
    x_int_1 = max(bbox1[0][0], bbox2[0][0])
    y_int_1 = max(bbox1[0][1], bbox2[0][1])
    x_int_2 = min(bbox1[1][0], bbox2[1][0])
    y_int_2 = min(bbox1[1][1], bbox2[1][1])

    # Compute area of intersection
    
    # Check if the bounding boxes are disjoint (no intersection)
    if x_int_2 - x_int_1 < 0 or y_int_2 - y_int_1 < 0:
        area_int = 0
    else:
        area_int = (x_int_2 - x_int_1 + 1) * (y_int_2 - y_int_1 + 1)
    
    # Compute area of both bounding boxes
    area_bbox1 = (bbox1[1][0] - bbox1[0][0] + 1) * (bbox1[1][1] - bbox1[0][1] + 1)
    area_bbox2 = (bbox2[1][0] - bbox2[0][0] + 1) * (bbox2[1][1] - bbox2[0][1] + 1)

    # Compute area of union
    area_union = float(area_bbox1 + area_bbox2 - area_int)

    # Compute and return IoU score
    score = area_int / area_union

    # Reject negative scores
    if score < 0:
        score = 0

    return score

def test_algorithm(detection_func, csv_file_path, template_file_path, swap=False):
    """
    Test a cone detection function and return the average score based on all the test images
    Input:
        detection_func: func; the cone detection function that takes the np.3darray
                as input and return (bottom, left, top, right) as output
        csv_file_path: string; the path to the csv file
        template_file_path: string, path to template file
        swap: Optional tag for indicating the template_file is really the background file
        For the map template matching, these need to be inverted
    Return:
        scores: dict; the score for each test image
    """
    # Keep track of scores
    scores = {}
    # Open test images csv
    with open(csv_file_path) as csvDataFile:
        csvReader = csv.reader(csvDataFile)
        # Iterate through all test images
        for row in csvReader:
            # Find image path and ground truth bbox
            img_path = row[0]
            bbox_true = ast.literal_eval(row[1])
            if not swap:
                img = cv2.imread(img_path)
                template = cv2.imread(template_file_path, 0)
            else:
                template = cv2.imread(img_path, 0)
                img = cv2.imread(template_file_path)
            # Detection bbox
            bbox_est = detection_func(img, template)
            score = iou_score(bbox_est, bbox_true)
            
            # Add score to dict
            scores[img_path] = score

    # Return scores
    return scores

def test_all_algorithms(csv_file_path, template_file_path, output_file_path, swap=False):
    """
    Test all algorithms and write score results to csv file
    Input:
        test_file_path: string; the path to the test file
    Output:
        test_scores: csvfile; the scores for each image for each algorithm
    """
    lookup_dict = dict({"template_matching": cd_template_matching,
                        "color_segmentation": cd_color_segmentation,
                        "SIFT": cd_sift_ransac})
    total_data = []
    for label in lookup_dict.keys():
        try:
            scores = test_algorithm(
                lookup_dict[label], csv_file_path, 
                template_file_path, swap=swap)
            data = [[label, img, score] for img, score in scores.items()]
        except:
            data = []

        total_data += data

    output_file = open(output_file_path, 'w')  
    with output_file:  
        writer = csv.writer(output_file)
        writer.writerows(total_data)

    print("Scores outputted to %s" % (output_file_path))

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Argument/s required")
    elif len(sys.argv) == 2:
        arg = sys.argv[1]
        if arg == "cone":
            scores = test_all_algorithms(cone_csv_path,
                cone_template_path,cone_score_path)
        elif arg == "map":
            scores = test_all_algorithms(localization_csv_path,
              localization_template_path, localization_score_path, swap=True)         
        elif arg == "citgo":
          scores = test_all_algorithms(citgo_csv_path,
              citgo_template_path, citgo_score_path)         
        else:
            print("Argument not recognized")

    elif len(sys.argv) == 3:
        scores = None
        algo_dict = dict({"color":cd_color_segmentation,
                            "sift":cd_sift_ransac,
                            "template":cd_template_matching})
        data_dict = dict({"cone":(cone_csv_path, cone_template_path),
            "map":(localization_csv_path, localization_template_path),
            "citgo":(citgo_csv_path, citgo_template_path)})
        swap = False
        args = sys.argv[1:3]
        if args[0] in {"cone", "map", "citgo"} and args[1] in {"color", "template", "sift"}:
            if args[0] == "map":
                swap = True
            scores = test_algorithm(algo_dict[args[1]],data_dict[args[0]][0],data_dict[args[0]][1], swap=swap)
        else:
            print("Argument/s not recognized")

        if scores:
            for (img, val) in scores.items():
                print((img, val))
    else:
        print("too many arguments")
