J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.0015;

%wheel radius
W = 0.05;


A = [0 1         0
     0 -b/J      K/J
     0 -K/L      -R/L];
B = [0
     0
    1/L];
C = [1   0   0];
D = 0;
motor_ss = ss(A,B,C,D)

Q = [0.001 0 0 0; 0 0 0 0;0 0 0 0;0 0 0 1];
R = [0.0000000000001];

K =lqi(motor_ss, Q, R)

Qk = 1;
Rk = 1;
Nk = 0;

[kalmf L P] = kalman(motor_ss, Qk, Rk, Nk);
L
kalmf

%Spring mass system controller
%spring constant
k = 4;
row = 0.4;
m = 5;

As = [0     1
      -k/m  -row/m];
Bs = [0
      1/m];
Cs = [k   0];
Ds = 0;

spring_ss = ss(As,Bs,Cs,Ds);

Qs = [0.4 0 0;0 0 0;0 0 1];
Rs = [0.000000000000001];

Ks = lqi(spring_ss, Qs, Rs)

