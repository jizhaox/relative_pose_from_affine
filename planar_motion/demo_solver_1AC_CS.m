clear;
load('data_planarmotion.mat')
[Yaw, AngleT] = solver_1AC_CS(Pi,Pj,Ac);

%%output difference
Yaw - Yaw_GT
AngleT - AngleT_GT