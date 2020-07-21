# Line and Plane based Indoor RGB-D Compass (LPIC)
This package provides a MATLAB implementation of CVPR 2018 paper: "Indoor RGB-D Compass from a Single Line and Plane" for the purpose of research and study only.
Note that this repository only includes simplified proposed 3-DoF rotation tracking example codes to understand how the LPIC works in structured environments.

![LPIC](https://github.com/PyojinKim/LPIC/blob/master/overview.png)


# 1. Goal
Our goal is to estimate 6-DoF camera motion with respect to the indoor structured environments.
The LPIC exploits line and plane primitives jointly to recognize the spatial regularities of orthogonal structured environments.
Lines from RGB images and surface normals from depth images are simultaneously used to perceive environmental regularities accurately and stably.
LPIC can track drift-free rotational motion while at least a single plane and a pair of lines parallel to the Manhattan world (MW) axes are visible. 
Given the absolute camera orientation, we recover the optimal translational motion, which minimizes de-rotated reprojection error.

![LPIC](https://github.com/PyojinKim/LPIC/blob/master/result.png)


# 2. Prerequisites
This package is tested on the MATLAB R2019b on Windows 7 64-bit.
This package depends on [mexopencv](https://github.com/kyamagu/mexopencv) library for keypoint processing, KLT tracking, and translation estimation.
cv.* functions in this package cannot run without mexopencv install in the MATLAB environment.
Please, build [mexopencv](https://github.com/kyamagu/mexopencv) in your OS first, and then run this package.
Some of the functions such as estimateSurfaceNormalGradient_mex.mexw64 are compiled as MEX file to speed up the computation.
You can use estimateSurfaceNormalGradient.m instead if you cannot compile MEX file.


# 3. Usage
* Download the ICL-NUIM dataset from https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html, 'of kt3' is recommended.

* Or, Use the ICSLRGBDdataset/rgbd_dataset_302_09_square3/ included in this package.

* Define 'datasetPath' correctly in your directory at setupParams_ICSL_RGBD.m file.

* Run LPIC_core/main_script_ICSL_RGBD.m, which will give you the 3D motion estimation result. Enjoy! :)


# 4. Publications
The approach is described and used in the following publications:

* **Indoor RGB-D Compass from a Single Line and Plane** (Pyojin Kim, Brian Coltin, and H. Jin Kim), CVPR 2018.

You can find more related papers at http://pyojinkim.com/_pages/pub/index.html.


# 5. License
The package is licensed under the MIT License, see http://opensource.org/licenses/MIT.

if you use LPIC in an academic work, please cite:

    @inproceedings{kim2018indoor,
      author = {Kim, Pyojin and Coltin, Brian and Kim, H Jin},
      title = {Indoor RGB-D Compass from a Single Line and Plane},
      year = {2018},
      booktitle = {IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
     }

