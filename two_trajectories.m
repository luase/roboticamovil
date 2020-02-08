%% One-dimensional case
% Defining the start and ending times
clc;
clear all;
ts = 0; 
tf = 10;
% S function
S = @(ts, tf) [ts^5    ts^4    ts^3   ts^2 ts 1;
               tf^5    tf^4    tf^3   tf^2 tf 1;
               5*ts^4  4*ts^3  3*ts^2 2*ts 1  0;
               5*tf^4  4*tf^3  3*tf^2 2*tf 1  0;
               20*ts^3 12*ts^2 6*ts   2    0  0;
               20*tf^3 12*tf^2 6*tf   2    0  0];
% Pose variable
pose = zeros(3, 3, 600);
% Desired boundary conditions for position, velocity and acceleration
ss = 0;
sf = 1;
sdots = 0;
sdotf = 0;
sddots = 0;
sddotf = 0;
% b
bcond = [ss sf sdots sdotf sddots sddotf]';
% A
A = S(ts, tf);
% Coeficients
scoef = A\bcond;
sdotcoef = scoef(1:5)'.*[5 4 3 2 1];
sddotcoef = scoef(1:4)'.*[20 12 6 2];
% Values over time
t = linspace(ts, tf, 100);
s = polyval(scoef, t);
sdot = polyval(sdotcoef, t);
sddot = polyval(sddotcoef, t);
% Multipoint trajectory
p0 = [3; 1];
p1 = [5; 1];
p2 = [6; 3];
p3 = [5; 5];
p4 = [3; 5];
p5 = [2; 3];

%% Movementk
% Initial conditions
sdots = 0;
sdotf = 0;
sddots = 0;
sddotf = 0;
%% p0 to p1
% setup
ssx = p0(1);
ssy = p0(2);
sfx = p1(1);
sfy = p1(2);
bcondx = [ssx sfx sdots sdotf sddots sddotf]';
bcondy = [ssy sfy sdots sdotf sddots sddotf]';
A = S(ts,tf);
% operations
scoefx = A\bcondx;
sdotcoefx = scoefx(1:5)'.*[5 4 3 2 1];
sddotcoefx = scoefx(1:4)'.*[20 12 6 2];
scoefy = A\bcondy;
sdotcoefy = scoefy(1:5)'.*[5 4 3 2 1];
sddotcoefy = scoefy(1:4)'.*[20 12 6 2];
% evaluations
t = linspace(ts, tf, 100);
x = polyval(scoefx, t);
xdot = polyval(sdotcoefx, t);
xddot = polyval(sddotcoefx, t);
y = polyval(scoefy, t);
ydot = polyval(sdotcoefy, t);
yddot = polyval(sddotcoefy, t);
%% p1 to p2
% setup
ssx = p1(1);
ssy = p1(2);
sfx = p2(1);
sfy = p2(2);
bcondx = [ssx sfx sdots sdotf sddots sddotf]';
bcondy = [ssy sfy sdots sdotf sddots sddotf]';
A = S(tf,tf*2);
% operations
scoefx = A\bcondx;
sdotcoefx = scoefx(1:5)'.*[5 4 3 2 1];
sddotcoefx = scoefx(1:4)'.*[20 12 6 2];
scoefy = A\bcondy;
sdotcoefy = scoefy(1:5)'.*[5 4 3 2 1];
sddotcoefy = scoefy(1:4)'.*[20 12 6 2];
% evaluations
t = linspace(tf, tf*2, 100);
x = [x polyval(scoefx, t)];
xdot = [xdot polyval(sdotcoefx, t)];
xddot = [xddot polyval(sddotcoefx, t)];
y = [y polyval(scoefy, t)];
ydot = [ydot polyval(sdotcoefy, t)];
yddot = [yddot polyval(sddotcoefy, t)];
%% p2 to p3
% setup
ssx = p2(1);
ssy = p2(2);
sfx = p3(1);
sfy = p3(2);
bcondx = [ssx sfx sdots sdotf sddots sddotf]';
bcondy = [ssy sfy sdots sdotf sddots sddotf]';
A = S(tf*2,tf*3);
% operations
scoefx = A\bcondx;
sdotcoefx = scoefx(1:5)'.*[5 4 3 2 1];
sddotcoefx = scoefx(1:4)'.*[20 12 6 2];
scoefy = A\bcondy;
sdotcoefy = scoefy(1:5)'.*[5 4 3 2 1];
sddotcoefy = scoefy(1:4)'.*[20 12 6 2];
% evaluations
t = linspace(tf*2, tf*3, 100);
x = [x polyval(scoefx, t)];
xdot = [xdot polyval(sdotcoefx, t)];
xddot = [xddot polyval(sddotcoefx, t)];
y = [y polyval(scoefy, t)];
ydot = [ydot polyval(sdotcoefy, t)];
yddot = [yddot polyval(sddotcoefy, t)];
%% p3 to p4
% setup
ssx = p3(1);
ssy = p3(2);
sfx = p4(1);
sfy = p4(2);
bcondx = [ssx sfx sdots sdotf sddots sddotf]';
bcondy = [ssy sfy sdots sdotf sddots sddotf]';
A = S(tf*3,tf*4);
% operations
scoefx = A\bcondx;
sdotcoefx = scoefx(1:5)'.*[5 4 3 2 1];
sddotcoefx = scoefx(1:4)'.*[20 12 6 2];
scoefy = A\bcondy;
sdotcoefy = scoefy(1:5)'.*[5 4 3 2 1];
sddotcoefy = scoefy(1:4)'.*[20 12 6 2];
% evaluations
t = linspace(tf*3, tf*4, 100);
x = [x polyval(scoefx, t)];
xdot = [xdot polyval(sdotcoefx, t)];
xddot = [xddot polyval(sddotcoefx, t)];
y = [y polyval(scoefy, t)];
ydot = [ydot polyval(sdotcoefy, t)];
yddot = [yddot polyval(sddotcoefy, t)];
%% p4 to p5
% setup
ssx = p4(1);
ssy = p4(2);
sfx = p5(1);
sfy = p5(2);
bcondx = [ssx sfx sdots sdotf sddots sddotf]';
bcondy = [ssy sfy sdots sdotf sddots sddotf]';
A = S(tf*4,tf*5);
% operations
scoefx = A\bcondx;
sdotcoefx = scoefx(1:5)'.*[5 4 3 2 1];
sddotcoefx = scoefx(1:4)'.*[20 12 6 2];
scoefy = A\bcondy;
sdotcoefy = scoefy(1:5)'.*[5 4 3 2 1];
sddotcoefy = scoefy(1:4)'.*[20 12 6 2];
% evaluations
t = linspace(tf*4, tf*5, 100);
x = [x polyval(scoefx, t)];
xdot = [xdot polyval(sdotcoefx, t)];
xddot = [xddot polyval(sddotcoefx, t)];
y = [y polyval(scoefy, t)];
ydot = [ydot polyval(sdotcoefy, t)];
yddot = [yddot polyval(sddotcoefy, t)];
%% p5 to p0
% setup
ssx = p5(1);
ssy = p5(2);
sfx = p0(1);
sfy = p0(2);
bcondx = [ssx sfx sdots sdotf sddots sddotf]';
bcondy = [ssy sfy sdots sdotf sddots sddotf]';
A = S(tf*5,tf*6);
% operations
scoefx = A\bcondx;
sdotcoefx = scoefx(1:5)'.*[5 4 3 2 1];
sddotcoefx = scoefx(1:4)'.*[20 12 6 2];
scoefy = A\bcondy;
sdotcoefy = scoefy(1:5)'.*[5 4 3 2 1];
sddotcoefy = scoefy(1:4)'.*[20 12 6 2];
% evaluations
t = linspace(tf*5, tf*6, 100);
x = [x polyval(scoefx, t)];
xdot = [xdot polyval(sdotcoefx, t)];
xddot = [xddot polyval(sddotcoefx, t)];
y = [y polyval(scoefy, t)];
ydot = [ydot polyval(sdotcoefy, t)];
yddot = [yddot polyval(sddotcoefy, t)];
%% Graphic
for i=1:600
    pos(:,:,i) = SE2(x(i), y(i), 0);
end