clear;
load('data_planarmotion.mat')
[Yaw, AngleT] = solver_1AC_LS(Pi,Pj,Ac);

%%Error
Yaw(4) - Yaw_GT
AngleT(4) - AngleT_GT