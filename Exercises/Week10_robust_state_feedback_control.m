%function K = robust_state_feedback_control
clc 

% State-space model
A = [1, -2; 1, 4];
B = [1; 0.1];

E = [1; 0];
H1 = [0.1, 0];
H2 = 0.3;

n = size(A,1);
m = size(B,2);


% Initialize description of LMIs
setlmis([]); 

% Specify matrix variables in LMIs
vbP = lmivar(1, [n,1]);
vbK = lmivar(2, [m,n]);
veps = lmivar(1, [1,1]);

% Specify term content of LMIs

% LMI #1
nlmi = 1;
lmiterm( [-nlmi, 1, 1,  vbP], 1, 1);

% LMI #2
nlmi = 2;
lmiterm( [-nlmi, 1, 1, veps], 1, 1);

% LMI #3
nlmi = 3;
lmiterm( [nlmi, 1, 1,   vbP], A, 1, 's');
lmiterm( [nlmi, 1, 1,   vbK], B, 1, 's'); 
lmiterm( [nlmi, 1, 1,  veps], E, E' );

lmiterm( [nlmi, 2, 1,   vbP], H1, 1);
lmiterm( [nlmi, 2, 1,   vbK], H2, 1);

lmiterm( [nlmi, 2, 2,  veps], -1, 1);

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
    disp('It is infeasible!!'); 
    K = NaN;
end 


