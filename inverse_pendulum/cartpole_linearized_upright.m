function [A, B] = cartpole_linearized_upright(p)
% Linearization about upright equilibrium
% state = [x; xdot; theta; thetadot]
% theta = 0 : upright

    M = p.M;
    m = p.m;
    l = p.l;
    g = p.g;
    b = p.b;

    A = [ 0,         1,                0, 0;
          0,      -b/M,          -(m*g)/M, 0;
          0,         0,                0, 1;
          0,   b/(M*l),   ((M+m)*g)/(M*l), 0 ];

    B = [ 0;
          1/M;
          0;
         -1/(M*l) ];
end