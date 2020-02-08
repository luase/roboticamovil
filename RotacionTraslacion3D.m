%% Rotación y traslación en 3D
P0 = [0 0 0]';
P1 = [1 1 1]';
theta1 = 0;
theta2 = pi/2;
T = 100;
% lspb significa linear segment with parabolic blends
[x, xd, xdd] = lspb(P0(1), P1(1), T);
[y, yd, ydd] = lspb(P0(2), P1(2), T);
[z, zd, zdd] = lspb(P0(3), P1(3), T);
[theta, thetad, thetadd] = lspb(theta1, theta2, T);

pos = zeros(4, 4, T);
for i = 1:T
    pos(:,:,i) = [rotz(theta(i))*rotx(theta(i)) [x(i) y(i) z(i)]'; 0 0 0 1];
end