
% state-space model
A = [-1, -2; 1, -4];
E = [1; 0.1];
H = [1, 0];
n = size(A,1);

% Initialize description of LMIs....
setlmis([]);

% Specify matrix variables in LMIs...
vP = lmivar(1, [n,1]);    % nxn, symmetric
veps = lmivar(1, [1, 1]); % scalar variable

% Specify term content of LMIs...

% LMI#1
lmiterm( [-1, 1, 1, vP], 1, 1);


% LMI #2
lmiterm( [2, 1, 1,    vP], 1, A, 's');
lmiterm( [2, 1, 1,  veps], H', H);

lmiterm( [2, 1, 2,    vP], 1, E);

lmiterm( [2, 2, 2,  veps], -1, 1);

% Compute solution
lmisys = getlmis;
options = [0,0,0,0,0];
target = 0;
[tmin, xfeas] = feasp(lmisys, options, target);

if ~isempty(xfeas) && tmin < 0
    disp('This system is robust stable!!');
    P = dec2mat(lmisys, xfeas, vP);
else
    disp('unstable!!');
    P = 0;
end
P