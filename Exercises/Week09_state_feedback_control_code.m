% Example 2.1. Using LMIs in Theorem 1, design a feasible
% state-feedback control gain K for (7) with A[], B[]

%function [K] = state_feedback_control()

clc, clearvars, close all

% Define state-space model
A = [1  -2; 1   4];
B = [1  0.1]'; % Transpose

n = size(A, 1);
m = size(B, 2);

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
    disp('P = ');disp(P);
    disp('K = ');disp(K);
else
    disp('It is NOT feasible!!');
    K = NaN;
end

%end % function

