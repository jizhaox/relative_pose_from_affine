clear;
close all;
load('data_solver_1AC_Essential.mat')
[R_recover, T_recover] = solver_1AC_Essential(Pi, Pj, Ac_rotated, Ri);

%%output difference
R_recover(:,:,4) - R_GT
T_recover(:,4) - T_GT