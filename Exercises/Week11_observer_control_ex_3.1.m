% Example 3.1. Design both control and observer gains K, L for
% given A, B, C, mu using Theorem 2

% function [K, L] = observer_control

clc, clearvars, close all

% State-space model 
A = [1  -2; 1   4];
B = [1  0.1]';
C = [0.1    0];
mu = 100;

n = size(A,1);
m = size(B,2); 
p = size(C,1);

% Initialize description of LMIs
setlmis([]); 

% Specify matrix variables in LMIs
vbP1  = lmivar(1, [n,1]);
vP2   = lmivar(1, [n,1]);
vbK = lmivar(2, [m,n]);
vbL = lmivar(2, [n,p]);

% Specify term content of LMIs
% LMI #1
nlmi = 1;
lmiterm( [-nlmi, 1, 1,  vbP1], 1, 1);

% LMI #2
nlmi = 2;
lmiterm( [-nlmi, 1, 1,  vP2], 1, 1);
 
% LMI #3
nlmi = 3;
lmiterm( [nlmi, 1, 1,  vbP1], A, 1, 's');
lmiterm( [nlmi, 1, 1,  vbK], B, 1, 's');
lmiterm( [nlmi, 1, 2,  vbK], -B, 1 );

lmiterm( [nlmi, 2, 2,  vbP1], -mu, 1, 's');
lmiterm( [nlmi, 2, 3,     0], mu );

lmiterm( [nlmi, 3, 3,   vP2], 1, A, 's');
lmiterm( [nlmi, 3, 3,   vbL], 1, C, 's'); 

% Compute solution
lmisys = getlmis;
options = [0,0,0,0,0];
target = 0;
[tmin, xfeas] = feasp(lmisys, options, target);

if ~isempty(xfeas) && tmin < 0 
    disp('It is feasible!!');
    bP1 = dec2mat(lmisys, xfeas, vbP1);
    bK = dec2mat(lmisys, xfeas, vbK);
    K  = bK * inv(bP1)

    P2 = dec2mat(lmisys, xfeas, vP2);
    bL = dec2mat(lmisys, xfeas, vbL);    
    L  = inv(P2)*bL
else
    disp('It is infeasible!!'); 
    K = NaN;
    L = NaN;
end 


