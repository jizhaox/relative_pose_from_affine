function [R_recover, T_recover] = solver_1AC_Essential(Pi, Pj, Ac_rotated, Ri)
thresh = 1e-8;

ui = Pi(1);
vi = Pi(2);
wi = Pi(3);
uj = Pj(1);
vj = Pj(2);
wj = Pj(3);

a1 = Ac_rotated(1,1);
a2 = Ac_rotated(1,2);
a3 = Ac_rotated(1,3);
a4 = Ac_rotated(2,1);
a5 = Ac_rotated(2,2);
a6 = Ac_rotated(2,3);

r1 =  Ri(1,1);
r2 =  Ri(1,2);
r3 =  Ri(2,1);
r4 =  Ri(2,2);
r5 =  Ri(2,3);
r6 =  Ri(3,1);
r7 =  Ri(3,2);
r8 =  Ri(3,3);

M = [ (ui*uj+ wi*wj) uj*vi (uj*wi - ui*wj)  ui*vj  vj*wi  vi*wj;
    (ui*a1 + wi*a3 + uj*r1 + wj*r6) (vi*a1 + uj*r3) (wi*a1 + uj*r6 - ui*a3 - wj*r1) (ui*a2 + vj*r1) (wi*a2 + vj*r6) (vi*a3 + wj*r3);
    (ui*a4 + wi*a6 + uj*r2 + wj*r7) (vi*a4 + uj*r4) (wi*a4 - ui*a6 + uj*r7 - wj*r2) (ui*a5 + vj*r2) (wi*a5 + vj*r7) (vi*a6 + wj*r4)];

N = null(M);

m1 = N(:,1);
m2 = N(:,2);
m3 = N(:,3);

E1 = [m1(1) m1(2) m1(3);
    m1(4) 0 m1(5);
    -m1(3) m1(6) m1(1)];
E2 = [m2(1) m2(2) m2(3);
    m2(4) 0 m2(5);
    -m2(3) m2(6) m2(1)];
E3 = [m3(1) m3(2) m3(3);
    m3(4) 0 m3(5);
    -m3(3) m3(6) m3(1)];

C = solver_coeff(m1(1), m1(2), m1(3), m1(4), m1(5), m1(6), m2(1), m2(2), m2(3), m2(4), m2(5), m2(6), m3(1), m3(2), m3(3), m3(4), m3(5), m3(6));

C2 = rref(C);
coeff = [0 C2(5, 7:10)] - [C2(6, 7:10) 0];
coeff = coeff / coeff(1);

A = [-coeff(2:end);
    1 0 0 0;
    0 1 0 0;
    0 0 1 0];
b = eig(A);

idx = abs(imag(b)) < thresh;
b = real(b(idx)); b = b(:);

num_sols = numel(b);
E_s = cell(num_sols, 1);
a = zeros(num_sols, 1);
for ii = 1:num_sols
    t = b(ii);
    a(ii) = -C2(6, 7:10)*[t^3; t^2; t; 1];
    tmp = a(ii)*E1 + b(ii)*E2 + E3;
    E_s{ii} = tmp / norm(tmp);
end

%% Recover relative pose from essential matrix
R_recover = zeros(3,3,num_sols);
T_recover = zeros(3,num_sols);

for ii = 1:num_sols
    Yaw = atand(E_s{ii}(1,1)/E_s{ii}(1,3));
    R_recover(:,:,ii) = [ cosd(Yaw)   0    -sind(Yaw);
                              0       1        0;
                          sind(Yaw)   0     cosd(Yaw)];
    
    tx = E_s{ii}(3,2);
    ty = E_s{ii}(1,1)/sind(Yaw);
    tz = -E_s{ii}(1,2);
    T_recover(:,ii)  = [tx; ty; tz];
end




