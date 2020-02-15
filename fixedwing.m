function xdot = fixedwing(x, v, uz, omega)

fixedwing = [v*cos(x(4));
             v*sin(x(4));
             uz;
             omega;];