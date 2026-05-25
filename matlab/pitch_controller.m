% Script fig04_02.m; LQR synthesis of fast pitch control w. a reaction
% wheel, & response to impulse that produces q(0+)=H(0+ =1==> ep*qw=1-q;
% x=[theta q]'; units: t in 1/sg, e=armature voltage in sg^2*R*Iy/N, 
% (q qw H/Iy) in sg; sg=(c+N^2/R)(1/Iy+1/J); ep=J/Iy; 7/91, 1/01, 5/28/05
% 

clear;
clc;
close all;

ep = 0.02;  % J / Iy, wheel to spacraft inertia ratio

% x_dot = F * x + G * u
%     y = H * x
% x = [theta; q]
F = [0 1; 0 -1];
G = [0 1]';
H = [1 0];

% lqr(F, G, Q, R, N)
% Compute the state-feedback control u = -K*x that minimizes
% the cost function:
%  
%        J = Integral {x'Qx + u'Ru + 2*x'Nu} dt
%  
% For the state dynamics x_dot = Fx + Gu, if:
% - (F, G) are stabilizable
% - [Q N;N' R] must be nonnegative definite
% 
k = lqr(F, G, H'*H, 1);

x0 = [0 1]';
t = [0:0.001:10]';
n = length(t);
un = ones(n,1);
u = zeros(n,1);

H1 = [eye(2); -k];
L1 = zeros(3,1); 
y = lsim(F-G*k, G, H1, L1, u, t, x0);
epqw = un - y(:,2);     % H - q
e = -un/(1+ep) + y(:,3);    % 4.16

%
figure(1);
subplot(211);
plot(t,y(:,1),t,y(:,2),t,un);grid;
axis([0 t(end) -.2 1.2]);
legend('\theta','q','H'); 
subplot(212);
plot(t,e,t,epqw);grid;
axis([0 t(end) -2 1.5]);
xlabel('\sigma*t'); legend('e','\epsilon*q_w')

% to save
%T = array2table([t y un], 'VariableNames', {'time', 'theta', 'q', 'e', 'H'});
%writetable(T, 'matlabDefault.csv');

% Model for CPP, PD
% x = [theta; q; H]
% u = [e; Qdist]
dT = 1 / 1000;
A = [0 1 0;0 -1 0;0 0 0];
B = [0 0;1 1;0 1];
C = [1 0 0];
D = [0 0];
sysC = ss(A,B,C,D);
sysD = ss(c2d(sysC, dT));

