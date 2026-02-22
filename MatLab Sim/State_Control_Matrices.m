%%  Calculating some matrices for my Rocket State Space Control
r = 0.33; % Moment Arm
I = 0.0857; %Moment of Inertia
avgT = 32.7; %Average Thrust

A = [0 1; 0 0];
B = [0; 1/I];

%%[A, B, C, D] = ssdata(linsys1);
Q = diag([10, 1]);
R = 2.5;

K = lqr(A, B, Q, R);
