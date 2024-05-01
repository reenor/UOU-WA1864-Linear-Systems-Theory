
% state-space model
A = [-1, -2; 1, -4];
N = [1; 0.1];
G = [1, 0];
n = size(A,1);
gm = 0.64;

% Initialize description of LMIs....
setlmis([]);

% Specify matrix variables in LMIs...
vP = lmivar(1, [n,1]);    % nxn, symmetric
vgm2 = lmivar(1, [1, 1]); % scalar variable

% Specify term content of LMIs...
% LMI#1
lmiterm( [-1, 1, 1, vP], 1, 1);

% LMI #2
lmiterm( [2, 1, 1,    vP], 1, A, 's');
lmiterm( [2, 1, 1,     0], G'*G);

lmiterm( [2, 1, 2,    vP], 1, N);

%lmiterm( [2, 2, 2,     0], -gm^2);
lmiterm( [2, 2, 2,     vgm2], -1,1);

% Compute solution
lmisys = getlmis;
options = [0,0,1,0,0];
target = 0;
[tmin, xfeas] = feasp(lmisys, options, target);

if ~isempty(xfeas) && tmin < 0
    disp('This system is Hinf stable!!');
    P = dec2mat(lmisys, xfeas, vP);
    gm2 = dec2mat(lmisys, xfeas, vgm2);
    disp(P);
    disp(sqrt(gm2));
else
    disp('unstable!!');
    P = NaN;
    gm2 = NaN;
end

