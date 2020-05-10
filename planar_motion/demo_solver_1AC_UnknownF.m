clear;
load('data_planarmotion.mat')

[Yaw, AngleT, FocalLength] = solver_1AC_UnknownF(Pi_pixel,Pj_pixel,Ac);

%%output difference
Yaw(1) - Yaw_GT
AngleT(1) - AngleT_GT
FocalLength(1) - FocalLength_GT