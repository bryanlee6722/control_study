function dx = cartpole_nonlinear(t, x, ufun, p)
% x = [x; xdot; theta; thetadot]
% theta = 0 ÀÌ upright
%
% Parameters:
% p.M : cart mass
% p.m : pendulum mass
% p.l : pendulum COM length
% p.g : gravity
% p.b : cart damping

    pos   = x(1);
    vel   = x(2);
    th    = x(3);
    thdot = x(4);

    force = ufun(t, x);

    M = p.M;
    m = p.m;
    l = p.l;
    g = p.g;
    b = p.b;

    S = sin(th);
    C = cos(th);

    % Equations of motion
    % theta = 0 at upright equilibrium
    denom = M + m - m*C^2;

    xdd = ( force - b*vel + m*l*thdot^2*S - m*g*S*C ) / denom;

    thdd = ( g*S - xdd*C ) / l;

    dx = [vel; xdd; thdot; thdd];
end