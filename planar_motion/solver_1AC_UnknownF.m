function [Yaw, AngleT, FocalLength] = solver_1AC_UnknownF(Pi_pixel,Pj_pixel,Ac)

%% Build the equation
data(1:4) = [Pi_pixel(1); Pi_pixel(2); Pj_pixel(1); Pj_pixel(2)];
data(5:8) = [Ac(1,1); Ac(1,2); Ac(2,1); Ac(2,2)];

%% Solving yaw angle, translation direction and focal length
solver = OneAC_unknown_f(data);
I = find(not(imag(solver(1,:))));
solver1 = solver(:,I);

if isempty(solver1)
    Yaw = [];
    AngleT = [];
    FocalLength = [];
    return;
else
    AngleT = zeros(1,size(I,2));
    Yaw = zeros(1,size(I,2));
    FocalLength = zeros(1,size(I,2));
    for i = 1:size(solver1,2)
        AngleT(1,i) = atan2(solver1(3,i),solver1(4,i))/pi*180;
        Yaw(1,i) = atan2(solver1(1,i),solver1(2,i))/pi*180 + AngleT(1,i);
        FocalLength(1,i) = 1/solver1(5,i);
    end
end




