%%%%%%% Paramters %%%%%%%
K = 120 % in N
w = 15 % in rad/sec
Jr = 0.03 % in kg* m2
Jp = 0.03 % in kg* m2
M = 1.4 % in kg
Ky = 4 % in N*m
Jyaw = 0.04 % in kg*m2
L = 0.2 % in m
r = .0025 % roll angle
p = .0025 % pitch angle

%%%%%%%% Part 2 %%%%%%%%  (also shows the matrices which is part 1 as well)

%Pitch and Roll
A1 = [0 1 0 0; 0 0 (K*L)/Jr 0; 0 0 -w 0; 1 0 0 0]
B1 = [0; 0; w; 0]
control_1 = ctrb(A1,B1)
rank_control_1 = det(control_1) % Not 0, so is controllable
C1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]
observ_1 = obsv(A1, C1)
rank_observ_1 = rank(observ_1) % rank = 4, so is observable

%Z-axis
A2 = [0 1 0 0; 0 0 (4*cos(r)*cos(p)*K)/M 0; 0 0 -w 0; 1 0 0 0]
B2 = [0; 0; w; 0;]
control_2 = ctrb(A2,B2)
rank_control_2 = det(control_2) % Not 0, so is controllable
C2 = C1
observ_2 = obsv(A2, C2)
rank_observ_2 = rank(observ_2) % rank = 4 so is observable

%X-axis
A3 = [0 1 0 0; 0 0 (4*sin(p)*K)/M 0; 0 0 -w 0; 1 0 0 0]
B3 = [0; 0; w; 0;]
control_3 = ctrb(A3,B3)
rank_control_3 = det(control_3) % Not 0, so is controllable
C3 = C2
observ_3 = obsv(A3, C3)
rank_observ_3 = rank(observ_3) % rank = 4 so is observable

%Y-axis
A4 = [0 1 0 0; 0 0 (-4*sin(r)*K)/M 0; 0 0 -w 0; 1 0 0 0]
B4 = [0; 0; w; 0;]
control_4 = ctrb(A4,B4)
rank_control_4 = det(control_4) % Not 0, so is controllable
C4 = C3
observ_4 = obsv(A4, C4)
rank_observ_4 = rank(observ_4) % rank = 4 so is observable

%Yaw-axis
A5 = [0 1; 0 0]
B5 = [0; Ky/Jyaw]
control_5 = ctrb(A5,B5)
rank_control_5 = det(control_5) % Not 0, so is controllable
C5 = [1 0; 0 1]
observ_5 = obsv(A5, C5)
rank_observ_5 = rank(observ_5) % rank = 2 so is observable

%%%%%%%% Part 3 %%%%%%%%

% calculates the optimal gain matrix K, 
% the solution S of the associated algebraic Riccati equation and the closed-loop poles P
Q1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]
Q2 = Q1
Q3 = Q2
Q4 = Q3
Q5 = [1 0; 0 1]
R= 1

%Pitch and Roll
[K1,S1,P1] = lqr(A1,B1,Q1,R) % Displays gain K1 matrix, solution to Ricatti S1 matrix , and closed loop poles P1

%Z-axis
[K2,S2,P2] = lqr(A2,B2,Q2,R) % Displays gain K2 matrix, solution to Ricatti S2 matrix , and closed loop poles P2

%X-axis
[K3,S3,P3] = lqr(A3,B3,Q3,R) % Displays gain K3 matrix, solution to Ricatti S3 matrix , and closed loop poles P3

%Y-axis
[K4,S4,P4] = lqr(A4,B4,Q4,R) % Displays gain K4 matrix, solution to Ricatti S4 matrix , and closed loop poles P4

%Yaw-axis
[K5,S5,P5] = lqr(A5,B5,Q5,R) % Displays gain K5 matrix, solution to Ricatti S5 matrix , and closed loop poles P5

%%%%%%%% Part 4 %%%%%%%%

D=[0] % sets D to 0 since not stated

%Pitch and Roll
sys1 = ss(A1-B1*K1,B1,C1, D);
step(sys1)


%Z-axis
sys2 = ss(A2-B2*K2,B2,C2, D);
step(sys2)

%X-axis
sys3 = ss(A3-B3*K3,B3,C3, D);
step(sys3)

%Y-axis
sys4 = ss(A4-B4*K4,B4,C4, D);
step(sys4)

%Yaw-axis
sys5 = ss(A5-B5*K5,B5,C5, D);
step(sys5)
