function estRes
 
% System matrices and initial condition
A = [-1, -2; 1, -2];
B = [1; 0.1];
C = [1, 0]; 

n = size(A,1);
m = size(B,2); 
p = size(C,1);

x0 = [0.1 ; -0.1];
 

%observer gain
L = [ 1.2266;     2.0277];

% Responds....
kMax = 10000;
dt = 0.001; 
Id = eye(2);

x(:,1) = x0;
hx(:,1) = [0; 0];

for k=1:kMax  
    u(:, k) = rand(1);
    y(:, k) = C*x(:,k);
    x(:, k+1) = (Id + A*dt)*x(:, k) + B*dt*u(:,k);

    hy(:, k) = C*hx(:,k);
    hx(:, k+1) = (Id + A*dt)*hx(:, k) + B*dt*u(:,k) - L*dt*(y(:,k) - hy(:,k));    
end  

%
figure

subplot(1,2,1)
title('(a)')
plot( [0:kMax]*dt, x(1,:), 'k', 'linewidth', 1.5);
hold on
plot( [0:kMax]*dt, hx(1,:), 'r:', 'linewidth', 2.5);
xlabel('Time (k)')
grid on

subplot(1,2,2)
title('(b)')
plot( [0:kMax]*dt, x(2,:), 'k', 'linewidth', 1.5);
hold on
plot( [0:kMax]*dt, hx(2,:), 'r:', 'linewidth', 2.5);
xlabel('Time (k)')
grid on

 
 