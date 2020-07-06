# README

This package is a MATLAB implementation of the minimal solutions for relative pose with a single affine correspondence [1]. It includes the following solvers.

* **under planar motion assumption**

     solver for planar motion with calibrated camera: closed-form solver (1AC_CS solver) and least-squares solver (1AC_LS solver).
	 
     solver for planar motion and unknown focal length: closed-form slover (1AC_UnknownF solver).
	 
* **with known vertical direction**

     solver for camera motion with known vertical direction: closed-form slover (1AC_Essential solver).

**Authors:** [Ji Zhao](https://sites.google.com/site/drjizhao) and Banglei Guan

# Reference

[1] Banglei Guan, Ji Zhao, Zhang Li, Fang Sun, and Friedrich Fraundorfer. [**Minimal Solutions for Relative Pose with a Single Affine Correspondence**](https://arxiv.org/pdf/1912.10776.pdf). IEEE Conference on Computer Vision and Pattern Recognition. 2020.

If you use this package in an academic work, please cite:

    @inproceedings{guan2020minimal,
      title={Minimal Solutions for Relative Pose with a Single Affine Correspondence},
      author={Guan, Banglei and Zhao, Ji and Li, Zhang and Sun, Fang and Fraundorfer, Friedrich},
      booktitle={IEEE Conference on Computer Vision and Pattern Recognition},
      pages={1929--1938},
      year={2020}
     }

# 1AC_CS solver

A closed-form solver for the relative pose estimation for planar motion with calibrated camera. It outputs yaw angle and translation direction.
* **File**:  src/planar_motion/solver_1AC_CS.m

* **API**:  [Yaw, AngleT] = solver_1AC_CS(Pi, Pj, Ac);

* **Input data for Demo**:

     Pi (3\*1 matrix): normalized image coordinates of a feature point in view i.

     Pj (3\*1 matrix): normalized image coordinates of a feature point in view j.

     Ac (2\*2 matrix): the corresponding 2\*2 local affine transformation of a feature point.

* **Demo**:  src/planar_motion/demo_solver_1AC_CS.m


# 1AC_LS solver

A least-squares solver for the relative pose estimation for planar motion with calibrated camera. It outputs a maximum of 8 solutions, including yaw angle and translation direction.
* **File**:  src/planar_motion/solver_1AC_LS.m

* **API**:  [Yaw, AngleT] = solver_1AC_LS(Pi, Pj, Ac);

* **Input data for Demo**:

     Pi (3\*1 matrix): normalized image coordinates of a feature point in view i.

     Pj (3\*1 matrix): normalized image coordinates of a feature point in view j.

     Ac (2\*2 matrix): the corresponding 2\*2 local affine transformation of a feature point.

* **Demo**:  src/planar_motion/demo_solver_1AC_LS.m


# 1AC_UnknownF solver

A closed-form solver for the relative pose estimation for planar motion and unknown focal length. It outputs a maximum of 6 solutions, including yaw angle, translation direction and focal length.
* **File**:  src/planar_motion/solver_1AC_UnknownF.m

* **API**:  [Yaw, AngleT, FocalLength] = solver_1AC_UnknownF(Pi_pixel, Pj_pixel, Ac);

* **Input data for Demo**:

     Pi_pixel (3\*1 matrix): pixel coordinates of a feature point in view i.

     Pj_pixel (3\*1 matrix): pixel coordinates of a feature point in view j.

     Ac (2\*2 matrix): the corresponding 2\*2 local affine transformation of a feature point.

* **Demo**:  src/planar_motion/demo_solver_1AC_UnknownF.m


# 1AC_Essential solver

A closed-form solver for the relative pose estimation with known vertical direction. It outpus a maximum of 4 solutions, including R_recover (3\*3 matrix) and T_recover (3\*1 matrix).
* **File**:  src/known_vertical_direction/solver_1AC_Essential.m

* **API**:  [R_recover, T_recover] = solver_1AC_Essential(Pi, Pj, Ac_rotated, Ri);

* **Input data for Demo**:

     Pi (3\*1 matrix): normalized image coordinates of a feature point in aligned view i.

     Pj (3\*1 matrix): normalized image coordinates of a feature point in aligned view j.

     Ac_rotated (3\*3 matrix): Ac_rotated= Transpose([Ac 0; 0 0])\*Transpose(Rj), where Ac is the corresponding 2\*2 local affine transformation of a feature point, Rj denotes 3\*3 rotation matrix for aligning the view j.

     Ri (3\*3 matrix): rotation matrix for aligning the view i.

* **Demo**:  src/known_vertical_direction/demo_solver_1AC_Essential.m

# License

This package is released under LPGL version 3 (or later). Please see the [LICENSE](https://github.com/jizhaox/relative_pose_from_affine/blob/master/LICENSE) for more information. 
