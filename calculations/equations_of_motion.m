clc; clear;

% in kg mm^2
I = [0.115662748,  -0.000520574,   0.012483367;
    -0.000520574,   1.602,        -0.000038399;
     0.012483367,  -0.000038399,   1.610];

m = 21;  % kg

M = zeros(6);
M(1,1) = m;
M(2,2) = m;
M(3,3) = m;
M(4:6,4:6) = I;

disp(M)

d_tube = 165.1;  % mm
l_tube = 852;  % mm
ld_ratio = l_tube / d_tube;

rho = 1e-6;



Ax = 21441.98683; 
Ay = d_tube * l_tube;
Az = d_tube * l_tube;

Cdx = 0.8;
Cdy = 1.17;
Cdz = 1.17;

Dx = (1/2) * rho * Cdx * Ax * 1000;
Dy = (1/2) * rho * Cdy * Ay * 1000;
Dz = (1/2) * rho * Cdy * Ay * 1000;
Dk = 0.2;
Dm = 4;
Dn = 4;

