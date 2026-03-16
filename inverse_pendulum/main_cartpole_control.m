clear; clc; close all;

%% Parameters
p.M = 1.0;
p.m = 0.1;
p.l = 0.5;
p.g = 9.81;
p.b = 0.0;

%% Initial condition
x0 = [0; 0; deg2rad(30); 0];

%% Simulation time
tspan = [0 10];
opts = odeset('RelTol',1e-8,'AbsTol',1e-9);

%% Linearized model
[A, B] = cartpole_linearized_upright(p);

disp('Open-loop eigenvalues:');
disp(eig(A));

%% Controllability check
Co = ctrb(A,B);
disp('Rank of controllability matrix:');
disp(rank(Co));

%% Pole placement
eigs_des = [-2.1; -2.2; -2.3; -2.4];
K = place(A, B, eigs_des);

disp('Feedback gain K:');
disp(K);

disp('Closed-loop eigenvalues:');
disp(eig(A - B*K));

%% Closed-loop input
u = @(t,x) -K*x;

%% Nonlinear closed-loop simulation
[t, X] = ode45(@(t,x) cartpole_nonlinear(t, x, u, p), tspan, x0, opts);

%% Linear closed-loop simulation
Acl = A - B*K;
sys_cl = ss(Acl, zeros(4,1), eye(4), zeros(4,1));

t_lin = linspace(tspan(1), tspan(2), 1000)';
Xlin = initial(sys_cl, x0, t_lin);
X_nl_interp = interp1(t, X, t_lin, 'pchip');

%% Plot
figure;
subplot(2,2,1);
plot(t_lin, X_nl_interp(:,1), 'LineWidth', 1.5); hold on;
plot(t_lin, Xlin(:,1), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Cart position x [m]');
legend('Nonlinear closed-loop','Linear closed-loop');

subplot(2,2,2);
plot(t_lin, X_nl_interp(:,2), 'LineWidth', 1.5); hold on;
plot(t_lin, Xlin(:,2), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Cart velocity xdot [m/s]');
legend('Nonlinear closed-loop','Linear closed-loop');

subplot(2,2,3);
plot(t_lin, rad2deg(X_nl_interp(:,3)), 'LineWidth', 1.5); hold on;
plot(t_lin, rad2deg(Xlin(:,3)), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('\theta [deg]');
legend('Nonlinear closed-loop','Linear closed-loop');

subplot(2,2,4);
plot(t_lin, rad2deg(X_nl_interp(:,4)), 'LineWidth', 1.5); hold on;
plot(t_lin, rad2deg(Xlin(:,4)), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('\theta dot [deg/s]');
legend('Nonlinear closed-loop','Linear closed-loop');

annotation('textbox', [0 0.95 1 0.05], ...
    'String', 'Cart-Pole Closed-Loop Response (Pole Placement)', ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold');

%% Animation
animate_cartpole(t, X, p);