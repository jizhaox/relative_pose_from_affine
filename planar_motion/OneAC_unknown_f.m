function sols = OneAC_unknown_f(data)
[C0,C1] = setup_elimination_template(data);
C1 = C0 \ C1;
RR = [-C1(end-2:end,:);eye(3)];
AM_ind = [1,2,3];
AM = RR(AM_ind,:);

if sum(isinf(AM(:)))> 0 ||  sum(isnan(AM(:)))> 0
    sols = [];
    return;
end

[V,D] = eig(AM);
scale = sqrt(diag(D).' ./ ((V(3,:).*V(3,:))));
V = V .* (ones(size(V,1),1)*scale);
x5 = V(3,:);
%warning('TODO: Extract remaining variables.');

n_sol = numel(x5);
ui = data(1);
vi = data(2);
uj = data(3);
vj = data(4);
a1 = data(5);
a2 = data(6);
a3 = data(7);
a4 = data(8);
sols = zeros(5, n_sol*2);
for ii = 1:n_sol
    g = x5(ii);
    A = [vi*g, vi*uj*g^2, vj*g, -ui*vj*g^2;
        0, a1*vi*g, a3, -(a3*ui+vj)*g;
        1, (a2*vi + uj)*g, a4, -a4*ui*g];
    if sum(isinf(A(:)))> 0 ||  sum(isnan(A(:)))> 0
        sols = [];
        return;
    end
    b = null(A); b = b(:);
    x1_x2 = b(1:2)/norm(b(1:2));
    x3_x4 = b(3:4)/norm(b(3:4));
    sols(:, ii*2-1:ii*2) = [x1_x2, -x1_x2;
        x3_x4, -x3_x4;
        g, g];
end


% Action =  x5^2
% Quotient ring basis (V) = x1*x4,x3*x4,x5,
% Available monomials (RR*V) = x1*x4*x5^2,x3*x4*x5^2,x5^3,x1*x4,x3*x4,x5,
function [coeffs] = compute_coeffs(data)
coeffs(1) = data(2)*data(3);
coeffs(2) = -data(1)*data(4);
coeffs(3) = data(2);
coeffs(4) = data(4);
coeffs(5) = data(2)*data(5);
coeffs(6) = -data(1)*data(7) - data(4);
coeffs(7) = data(7);
coeffs(8) = data(2)*data(6) + data(3);
coeffs(9) = -data(1)*data(8);
coeffs(10) = 1;
coeffs(11) = data(8);
coeffs(12) = -1;
function [C0,C1] = setup_elimination_template(data)
[coeffs] = compute_coeffs(data);
coeffs0_ind = [1,5,8,1,5,8,2,6,9,2,6,9,3,10,10,1,10,4,3,7,11,10,4,7,11,10,2,1,5,2,6,10,3,5,8,1,4,1,8,5,10,3,7,3,11,10,4,4,11,7,...
    10,8,5,9,6,10,3,6,9,2,4,7,2,9,6,12,12];
coeffs1_ind = [10,11,7,12];
C0_ind = [1,3,5,22,24,27,41,43,45,62,64,67,81,85,88,106,108,121,122,123,125,127,142,144,147,160,166,169,170,189,190,200,206,211,213,215,226,232,234,238,253,255,271,272,273,274,275,292,294,298,...
    299,316,317,336,337,339,349,351,353,355,369,370,372,374,378,388,400];
C1_ind = [16,36,37,59];
C0 = zeros(20,20);
C1 = zeros(20,3);
C0(C0_ind) = coeffs(coeffs0_ind);
C1(C1_ind) = coeffs(coeffs1_ind);

