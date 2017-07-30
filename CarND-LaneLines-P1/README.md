# **Finding Lane Lines on the Road** 

---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)


---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 7 steps. 
1- First, I cropped the image to the central portion of size 960x540. This is required for the "Challenging" video, which has a both a larger field of view than the other videos and also a larger resolution. Cropping the central portion of the third video makes all 3 videos similar in FoV.
2- I converted the images to grayscale
3- I applied a Gaussian noise to remove smooth the noisy gray gradient
4- I detected the edges (color gradient) by applying the canny function with 50 and 150 thresholds. Those thresholds have been selected empirically.
5- a region of interest, which consist in a trapezoid, is then used to filter the region of the image where is expected to find the road lines
6- the hough line function is applied to the image. The parameters have also been tuned empirically. a 1st order polynomial fit has been used to generate a single line that fits the left and right hough lines.
7- finally, the 1st order polynomial has been overlaid on top of the original image to visually show how the algorithms works



### 2. Identify potential shortcomings with your current pipeline

The proposed lane detection algorithm is very simple and it is expected to fail under many circumstances:
- the region of interest is fixed and it exclude line markings in case the car isn't well centered in the lane.
- Any noise in the region of interest that can generate color gradient (such as cars, road color change, guardrails etc...) can easily reesult in a completely wrong output of the algorithm
- using a first order polynomial to fit the lines isn't a good option during turns, where we should capture the curvature of the road as well. 


### 3. Suggest possible improvements to your pipeline

Those are possible improvements to the algorithm:
1- apply an homography to the image to have a top-view visualization. This will facilitate the curve fitting techniques, since all the points of the line are more homogeneously weighted
2- use a higher order polynomial as a fitting function. Usually a 3rd order polynomial is recommended.
3- since lane markings are either yellow or white, we extract isolate colors from the image before converting to a greyscale.
