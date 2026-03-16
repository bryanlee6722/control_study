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

%% Input force (open-loop)
u = @(t, x) 0.0;

%% Nonlinear simulation
opts = odeset('RelTol',1e-8,'AbsTol',1e-9);
[t, X] = ode45(@(t,x) cartpole_nonlinear(t, x, u, p), tspan, x0, opts);

%% Linearized model
[A, B] = cartpole_linearized_upright(p);
sys = ss(A, B, eye(4), zeros(4,1));

%% lsim용 균일 시간 벡터 생성
t_lin = linspace(tspan(1), tspan(2), 1000)';
U = zeros(size(t_lin));

%% 선형계 응답
Xlin = lsim(sys, U, t_lin, x0);

%% 비선형 결과도 같은 시간축으로 보간해서 비교
X_nl_interp = interp1(t, X, t_lin, 'pchip');

%% Plot
figure;
subplot(2,2,1);
plot(t_lin, X_nl_interp(:,1), 'LineWidth', 1.5); hold on;
plot(t_lin, Xlin(:,1), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Cart position x [m]');
legend('Nonlinear','Linear');

subplot(2,2,2);
plot(t_lin, X_nl_interp(:,2), 'LineWidth', 1.5); hold on;
plot(t_lin, Xlin(:,2), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Cart velocity xdot [m/s]');
legend('Nonlinear','Linear');

subplot(2,2,3);
plot(t_lin, rad2deg(X_nl_interp(:,3)), 'LineWidth', 1.5); hold on;
plot(t_lin, rad2deg(Xlin(:,3)), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('\theta [deg]');
legend('Nonlinear','Linear');

subplot(2,2,4);
plot(t_lin, rad2deg(X_nl_interp(:,4)), 'LineWidth', 1.5); hold on;
plot(t_lin, rad2deg(Xlin(:,4)), '--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('\theta dot [deg/s]');
legend('Nonlinear','Linear');

%% animation은 원래 nonlinear 결과 사용
animate_cartpole(t, X, p);