% Example 3.1. Using LMIs in Theorem 2, design a feasible LQR
% state-feedback control gain K for (7) with A[], B[], Q[], R constant

%function [K] = lqr_state_feedback_control()

% Define state-space model
A = [1  -2; 1   4];
B = [1  0.1]'; % Transpose

n = size(A, 1);
m = size(B, 2);

% Define cost function
% As the value of Q in the LQR control increases, the state response
% converges to the origin more quickly.
% However, increasing Q too much can also lead to instability or
% excessive control effort, so it often requires a balance and may involve
% tuning through simulation or experimentation.

%Q = 10000000000*eye(n); % Lead to NOT feasible

Q = 1*eye(n);
R = 10;

% Initialize description of LMIs
setlmis([]);

% Specify matrix variables in LMIs
vbP = lmivar(1, [n, 1]); % P bar belongs to R(nxn)
vbK = lmivar(2, [m, n]); % K bar belongs to R(mxn)

% Specify term content of LMIs
% LMI #1
lmiterm( [-1, 1, 1, vbP], 1, 1); % 0 < Pbar

% LMI #2
lmiterm( [2, 1, 1,  vbP], A, 1, 's'); % A*Pbar + Pbar*A', Pbar' = Pbar
lmiterm( [2, 1, 1,  vbK], B, 1, 's'); % B*Kbar + Kbar*B', Kbar' = Kbar

lmiterm( [2, 2, 1,  vbP], Q, 1); % Q*Pbar
lmiterm( [2, 2, 2,  0],   -Q); % -Q

lmiterm( [2, 3, 1,  vbK], R, 1); % R*Kbar
lmiterm( [2, 3, 3,  0],   -R); % -R

% Compute solution
lmisys = getlmis;
options = [0,0,0,0,0];
target = 0;
[tmin, xfeas] = feasp(lmisys, options, target);

if ~isempty(xfeas) && tmin < 0
    disp('It is feasible!!');
    bP = dec2mat(lmisys, xfeas, vbP);
    bK = dec2mat(lmisys, xfeas, vbK);
    P  = inv(bP);
    K  = bK * inv(bP);
    disp('K = ');disp(K);
else
    disp('It is NOT feasible!!');
    K = NaN;
end

%end % function

