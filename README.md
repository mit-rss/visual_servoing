# Lab 4: Vision

Welcome to Lab 4, where you will learn how to use the camera to allow the racecar to park in front of a colored cone and follow a line. 

In this lab, your team will do the following:
- Experiment/Prototype with several types of object detection algorithms
- Learn how to transform a pixel from an image to a real world plane
- Develop a parking controller to park your robot in front of an orange cone
- Extend your parking controller into a line following controller

### Lab Modules
This lab has a lot in it, so we are encouraging parallelization by breaking up the components of the lab into 4 distinct modules, which you will combine together. Each module tackles an interesting problem in computer vision/controls, and is designed to be implemented (initially) by itself. 
- Module 1: Cone Detection via Color Segmentation
- Module 2: Object Detection via Template Matching and SIFT
- Module 3: Transforming pixels to a plane via Homography
- Module 4: Writing a parking controller.

Here’s how they fit together. Modules 1 and 2 cover object detection algorithms. Comparing different algorithms will give you a better feel for what is out there. Module 2 will teach you how to convert a pixel to a plane in the real world. Combining 1 and 2 will tell you where a cone is relative to your robot. Module 3 will park a robot in front of a simulated cone. Bring in modules 1 and 2 and put it on hardware to park in real life. Now make some modifications to follow a line instead!

### Bringing it together:
With your modules in hand it is time to make your robot park in front of a cone. Here are suggested steps:
1. Write a ros node using Modules 1 and 3 that publishes the relative location of a cone in view of the ZED Camera. Make sure you can see the cone in rviz before trying to do control. You should now be able to move the cone around on the floor and accurately determine its position relative to the car using just the camera. Make sure to publish a Marker to RVIZ representing the cone. The rviz cone should appear where the real cone does.
2. Now bring in Module 4. Listen to the relative cone location and publish drive commands. Congratulations are in order when you can park successfully. 
3. Modify module 1 such that your robot can follow a line instead of a cone. Details in the module 1 handout.
4. Improve your line following controller to see how fast you can navigate a circular track. 

### Analysis:
We are also looking for a bit more in terms of experimental analysis in the lab than we have in the past. We are, in particular, looking for analysis of your vision algorithms and and controller.

Vision Analysis:
We wrote some code to test the Intersection over Union (IOU) scores of your vision algorithms on the three datasets provided. IOU is a measure of how accurate bounding boxes are, and is  a choice metric for analysis of object detection algorithms. Go into **computer_vision/**  and run:
- python cv_test.py citgo
- python cv_test.py cone
- python cv_test.py map
To test all three of your algorithms against our citgo, cone, and stata basement datasets respectively. Results will be outputted to .csv files in **scores/**. Some algorithms on some datasets won’t get any/good results. This is expected, and we would like to know why each works for what it does in your presentation.

Controller analysis: 
When you wrote the parking controller (module 4), you published error messages. Now it’s time to use **rqt_plot** to generate some plots. Try running the following experiments:
- Put a cone directly in front of the car, ~3-5 meters away. Your car should drive straight forward and stop in front of the cone. Show us plots of x-error and total-error over time, and be prepared to discuss.
- Run the car on one of our tracks, and check out the plots for any interesting error signals. Compare plots at different speeds, and see how error signals change with speed.
### Grading: /10
Technical implementation
- 1 point for satisfactory completion of module 1
- 1 point for satisfactory completion of module 2
- 1 point for satisfactory completion of module 3
- 1 point for satisfactory completion of module 4
- 1 point for successful integration of the 4 components
Evaluation(presentation):
- 2 points for explaining vision algorithm strengths and weaknesses. Why does each algorithm perform as it does on each dataset?
- 1 point for explaining the homography transformation. How do we convert pixels to plane coordinates?
- 1 point for demonstrating and explaining performance of the parking controller. Make sure you mention your method for tuning the controller gains. Hint: include error plots from **rqt_plot**
- 1 point for demonstrating and explaining performance of the line-follower. Make sure you mention your method for tuning the controller gains. Hint: include error plots from **rqt_plot**
Bonus:
- +1 point for the fastest line follower on the circular track. Have a TA record your time. 
