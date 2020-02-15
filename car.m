% Differential steering car simulation
% (c) Universidad de Guanajuato 2019
% Author: Juan-Pablo Ramirez-Paredes
%         <jpi.ramirez@ugto.mx>
% Robotica Movil

% x es un vector con los estados del modelo
% r es el radio
% L es la distancia entre 
function xdot = car(x, r, L, ur, ul)

xdot = [0.5*r*(ul+ur)*cos(x(3)); 
        0.5*r*(ul+ur)*sin(x(3));
        (r/L)*(ur-ul)];