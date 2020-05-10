function [Yaw, AngleT] = solver_1AC_LS(Pi,Pj,Ac)

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
solver = OneAC_LS(reshape(CoefficientMatrix',[1,12]));
I = find(not(imag(solver(1,:))));
solver1 = solver(:,I);

if isempty(solver1)
    Yaw = [];
    AngleT = [];
    return;
else
    AngleT = zeros(1,size(I,2));
    Yaw = zeros(1,size(I,2));
    for i = 1:size(solver1,2)
        AngleT(1,i) = atan2(solver1(3,i),solver1(4,i))/pi*180;
        Yaw(1,i) = atan2(solver1(1,i),solver1(2,i))/pi*180 + AngleT(1,i);
    end
end




