%% EE362_PROJECT_25
%% DC Motor Control Project


clc;
clear;
close all;

%% Motor Parameters
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;

s = tf('s');

%% Open-loop Transfer Function
P_motor = K / ((J*s + b)*(L*s + R) + K^2);

%% Linear System Analyzer - Step Response
linearSystemAnalyzer('step', P_motor, 0:0.1:5);

%% Open-loop Frequency Response
figure;
bode(P_motor);
grid on;
title('Open-loop Frequency Response');

%% Open-loop Root Locus
figure;
rlocus(P_motor);
grid on;
title('Open-loop Root Locus');

%% Proportional Controller
Kp = 100;
C = pid(Kp);

sys_cl = feedback(C*P_motor,1);

t = 0:0.01:5;
figure;
step(sys_cl,t);
grid on;
title('Step Response with Proportional Control');

%% PID Controller - Small Ki and Kd
Kp = 75;
Ki = 1;
Kd = 1;

C_pid_min = pid(Kp,Ki,Kd);
sys_pid_min = feedback(C_pid_min*P_motor,1);

figure;
step(sys_pid_min,t);
grid on;
title('PID Response with Small Ki & Kd');

%% PID Controller - Large Ki and Kd
Kp = 100;
Ki = 200;
Kd = 10;

C_pid_max = pid(Kp,Ki,Kd);
sys_pid_max = feedback(C_pid_max*P_motor,1);

figure;
step(sys_pid_max,t);
grid on;
title('PID Response with Large Ki & Kd');

%% Root Locus with PID (Large Gains)
figure;
rlocus(C_pid_max*P_motor);
grid on;
title('Root Locus with PID');

%% Frequency Response with PID (Small Gains)
figure;
bode(C_pid_max*P_motor);
grid on;
title('Frequency Response with PID');
