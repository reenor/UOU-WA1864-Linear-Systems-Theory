function L = luenberger_observer
clc 

%% State-space model
%A = [1, -2; 1, 4];
A = [-1, -2; 1, -2];
B = [1; 0.1];
C = [1, 0]; 

n = size(A,1);
m = size(B,2); 
p = size(C,1);


%% Initialize description of LMIs
setlmis([]); 

%% Specify matrix variables in LMIs
vP  = lmivar(1, [n,1]);
vbL = lmivar(2, [n,p]);

%% Specify term content of LMIs

% LMI #1
nlmi = 1;
lmiterm( [-nlmi, 1, 1,  vP], 1, 1);
 
% LMI #3
nlmi = 2;
lmiterm( [nlmi, 1, 1,    vP], 1, A, 's');
lmiterm( [nlmi, 1, 1,   vbL], 1, C, 's'); 

%% Compute solution 
lmisys = getlmis;
options = [0,0,0,0,0];
target = 0;
[tmin, xfeas] = feasp(lmisys, options, target);

if ~isempty(xfeas) && tmin < 0 
    disp('It is feasible!!');
    P = dec2mat(lmisys, xfeas, vP);
    bL = dec2mat(lmisys, xfeas, vbL);    
    L  = inv(P)*bL;
else
    disp('It is infeasible!!'); 
    L = NaN;
end 


