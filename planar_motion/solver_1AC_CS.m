function [Yaw, AngleT] = solver_1AC_CS(Pi,Pj,Ac)

u1 = Pi(1);
v1 = Pi(2);
u2 = Pj(1);
v2 = Pj(2);

a1 = Ac(1,1);
a2 = Ac(1,2);
a3 = Ac(2,1);
a4 = Ac(2,2);

%% Build the equation
CoefficientMatrix(1,:) = [v1 v1*u2 v2 -u1*v2];
CoefficientMatrix(2,:) = [0 a1*v1 a3 -(a3*u1+v2)];
CoefficientMatrix(3,:) = [1 (a2*v1+u2) a4 -a4*u1];

%% Solving yaw angle and translation direction 
[solver, D] = eig(CoefficientMatrix'*CoefficientMatrix);
AngleT = atan2(solver(3,1),solver(4,1))/pi*180;
Yaw = atan2(solver(1,1),solver(2,1))/pi*180 + AngleT;


