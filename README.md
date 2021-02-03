# 2D-Collision-Prevention
Camera based 2D feature tracking to prevent collision on the road using Open CV 2D Features framework. This implementation is comprised of main three components - Detectors, Descriptors and Matching algorithm. By using Open CV , the implemented 7 detectors and 6 descriptors including binary string based ones and hog family one are as follows: 
* Detectors: "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"
* Desctiptors:  "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"
And 6 descriptors are implemented along with 2 type of matchers (Brute force way and FLNN alogorithm) , (nearest neighbor and k's nearest neighbor algorithm) 

## Implementation Approach

### 1. Data Buffer Optimization
In case of processing a large image sequence with several thousand images and Lidar point clouds over night - this would push the memory of processing computer to its limit and eventually slow down the entire program. So in order to prevent this, it would be better to hold a certain number of images in memory so that when a new one arrives, the oldest one is deleted from one end of the vector and the new one is added to the other end. Data buffer is a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This is implmented by pushing in new elements on one end and removing elements on the other end. So that when the image data grows, memeory holding image data remains consistant in a manageble size. 

### 2 Keypoint Detection
A selection of seven detectors, which are Shi-Tomasi, HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT, is implemented. All these detectors are selectable by setting the string `detectorType` to the respective name. Based on detail implementation pattern, three functions can be explicitely called. These fuctions are declared in the header file regarding to its call parameters.
* Shi-Tomasi detector : `void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)`
* Harris conor detection : `void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)`
* Other detectors : `void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)` implemented the rest firve dectors including FAST, BRISL, ORB, AKAZE and SIFT selectable by setting the string parameter `detectorType`.

### 3. Keypoint Removal
As for a collision detection system, keypoints on the preceding vehicle are of special interest. Therefore, in order to exploere a combination of dector/descriptor pair, it's better to discard feature points that are not located on the preceding vehicle and only focussing on targeted area(but just for the evaluation to pick the best combo) So I removed all keypoints outside of a pre-defined rectangle (Box parameters are : cx = 535, cy = 180, w = 180, h = 150) in which the coordinates are based on the `cv::Rect`datatype in OpenCV and only stored the keypoints within the rectangle for further processing by checking whether the pre-defined rectangle(`cv::Rect`) contains(`contains`) the keypoints or not. 

### 4. Keypoint Descriptors
> `void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)`

Implemented six different descriptors BRISK, BRIEF, ORB, FREAK, AKAZE and SIFT and made them selectable by setting a string 'descriptorType' accordingly. As for these descriptors, both Binary string based descriptors (BRIEF, FAST, BRISK, ORB, AKAZE) and one of HOG Family detector, SIFT are implemented and these different detectors are selectable by passing a string parameter accordingly. SIFT(SURF also) is patented, and thus could not be freely used in a commercial context. As for an older version of the OpenCV installed, use `#include <opencv2/xfeatures2d/nonfree.hpp>` in order to use both algorithms. In versions of the OpenCV >= 4.3, SIFT and SURF can be used from `#include <opencv2/xfeatures2d.hpp>`

### 5. Descriptor Matching
> `void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)`

Implemented Brute Force and FLANN matching algorithm as well as nearest neighbor and k-nearest neighbor selection which are selectable using the respective strings `matcherType` and `selectorType` . In order to use these parameters, set the string to use Brute-Force matching combined with Nearest-Neighbor selection or k-nn selection and use FLANN instead of Brute-Force matching
* `matcherType` : "MAT_BF" , "MAT_FLANN" 
    * As for Binary Matcher ("MAT_BF"), since `SIFT` is HOA fmaily descriptors, so that `cv::NORM_L2` is used and the others are binary string based descriptors so that `cv::NORM_HAMMING` is used for as distance measurement parameter `normType`. Detailes are guided in the [OpenCV documentation] (https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html)
    * As for FLANN matching algorithm ("MAT_FLANN"), conversion binary descriptors to floating point is added due to a bug in current OpenCV version
* `selectorType`: "SEL_NN", "SEL_KNN"

### 6. Descriptor Distance Ratio
Applied the K-Nearest-Neighbor matching algorithm to implement the descriptor distance ratio test, which looks at the ratio of best vs. kth best match to decide whether to keep an associated pair of keypoints and outputs the number of removed bad keypoint matches. Here, distance ratio is set to `minDescDistRatio=0.8`.

## Performance Evaluation 

### keypoints_file.csv 
A CSV file stores the number of keypoints on the preceding vehicle for all 10 images and the distribution of their neighborhood size for all 7 detectors

### matched_keypoints.csv
A CSV File stores the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors that meets following conditions (35 combos). In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.
* AKAZE descriptros only works with AKAZE detectors.
* ORB descriptros does not work with SIFT detectors.


### log_time.csv
A CSV File logs the time it takes for keypoint detection and descriptor extraction. 

### Recommendation for detector/descriptor combinations
Based on above data, The TOP3 detector / descriptor combinations are recommended as the best choice for detecting keypoints on vehicles as follows:
* 
* 
* 

### Rerefence 
[Harris Detector](https://docs.opencv.org/3.4/d4/d7d/tutorial_harris_detector.html)
[Brute Force Matcher](https://docs.opencv.org/3.4/dc/dc3/tutorial_py_matcher.html)
[Detector and Descriptor](https://docs.opencv.org/2.4/modules/features2d/doc/feature_detection_and_description.html)

