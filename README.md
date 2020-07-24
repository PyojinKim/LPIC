# Line and Plane based Indoor RGB-D Compass (LPIC)
This package provides a MATLAB implementation of CVPR 2018 paper: "Indoor RGB-D Compass from a Single Line and Plane" for the purpose of research and study only.
Note that this repository only includes simplified proposed 3-DoF rotation tracking example codes to understand how the LPIC works in structured environments.

![LPIC](https://github.com/PyojinKim/LPIC/blob/master/overview.png)


# 1. Goal
Our goal is to estimate 3-DoF camera orientation with respect to the spatial regularities of indoor structured environments.
The proposed LPIC estimates absolute 3-DoF camera orientation from only a single line and a single plane, which corresponds to the theoretical minimal sampling for 3-DoF rotation estimation.
Our algorithm requires a plane and a line on the plane aligned with the Manhattan world (MW) to be visible, which is typically the case in most indoor environments.

![LPIC](https://github.com/PyojinKim/LPIC/blob/master/result.png)


# 2. Prerequisites
This package is tested on the MATLAB R2019b on Windows 7 64-bit.
Some of the functions such as estimateSurfaceNormalGradient_mex.mexw64 are compiled as MEX file to speed up the computation.
You can use estimateSurfaceNormalGradient.m instead if you cannot compile MEX file in your OS.


# 3. Usage
* Download the TUM-RGBD dataset from https://vision.in.tum.de/data/datasets/rgbd-dataset.

* Or, Use the TUMRGBDdataset/rgbd_dataset_freiburg3_structure_notexture_far/ included in this package.

* Define 'datasetPath' correctly in your directory at setupParams_TUM_RGBD.m file.

* Run LPIC_core/main_script_TUM_RGBD.m, which will give you the 3-DoF camera orientation tracking result. Enjoy! :)


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

