function stateRes
 
% System matrices and initial condition
A = [1, -2; 1, 4];
B = [1; 0.1];

x0 = [1 ; -1];
 

%LQR control gain
%(1) Q=[1,0;0,1], R=10
K = [-10.0239  -52.4001];

%(2) Q=[100,0;0,100], R=10
%K = [-15.2377 -108.4757];

% Responds....
kMax = 10000;
dt = 0.001; 
Id = eye(2);

x(:,1) = x0;
for k=1:kMax   
    u(:, k) = K*x(:, k);    
    x(:, k+1) = (Id + dt*A)*x(:, k)  + B*dt*u(:,k);
end  

%
figure

subplot(1,2,1)
title('(a)')
plot( [0:kMax]*dt, x(1,:), 'k', 'linewidth', 1.5);
hold on
plot( [0:kMax]*dt, x(2,:), 'k:', 'linewidth', 2.5);
xlabel('Time (k)')
grid on

subplot(1,2,2)
title('(b)')
plot([0:kMax-1]*dt, u, 'k', 'linewidth', 2);
xlabel('Time (k)')
grid on

 
 