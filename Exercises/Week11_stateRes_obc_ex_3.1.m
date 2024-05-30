%function stateRes_obc

%clc, clearvars, close all

% System matrices and initial condition
A = [1  -2; 1   4];
B = [1  0.1]';
C = [0.1    0];

% Initial condition
x0 = [1; -1];
 
% Observer-based control design
%(1) via separation principle
% K = [-7.4147  -108.0534];
% L = [-80.9385 291.6843]';

%(2) via Lyapunov stability approach with mu = 100
K = [-48.5911   -280.6373];
L = [-89.3317   330.6961]';

% Responds....
kMax = 10000;
dt = 0.001; 
Id = eye(2);

x(:,1) = x0;
hx(:,1) = [0; 0];

for k=1:kMax  
    u(:, k) = K*hx(:,k);
    y(:, k) = C*x(:,k);
    hy(:, k) = C*hx(:,k);

    x(:, k+1) = (Id + A*dt)*x(:,k) + B*dt*u(:,k);    
    hx(:, k+1) = (Id + A*dt)*hx(:,k) + B*dt*u(:,k) - L*dt*(y(:,k) - hy(:,k));    
end  

%
figure
title('(a)')
hold on
plot(x(1,:), x(2,:), 'k', 'linewidth', 1.5) 
xlabel('x_1')
ylabel('x_2')
grid on
 
 
 