function K = state_feedback_control
clc 

%% State-space model
A = [1, -2; 1, 4];
B = [1; 0.1];

n = size(A,1);
m = size(B,2); 

%% Initialize description of LMIs
setlmis([]); 

%% Specify matrix variables in LMIs
vbP = lmivar(1, [n,1]);
vbK = lmivar(2, [m,n]);

%% Specify term content of LMIs

% LMI #1
lmiterm( [-1, 1, 1,  vbP], 1, 1);

% LMI #2
lmiterm( [2, 1, 1,   vbP], A, 1, 's');
lmiterm( [2, 1, 1,   vbK], B, 1, 's'); 

%% Compute solution 
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
else
    disp('It is infeasible!!'); 
    K = NaN;
end 


