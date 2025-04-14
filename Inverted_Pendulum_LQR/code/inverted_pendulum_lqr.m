%% Inverted Pendulum on a Cart - LQR Control Simulation
% Description:
%   Simulates an inverted pendulum on a cart using a linearized model.
%   Designs an LQR controller to balance the pendulum upright.
%   Includes plots of cart position, pendulum angle, and control input.

clear; clc; close all;


%%  Define Physical Parameters


M = 1.0;      % Mass of the cart (kg)
m = 0.1;      % Mass of the pendulum (kg)
l = 0.5;      % Distance from pivot to center of mass (m)
g = 9.81;     % Acceleration due to gravity (m/s^2)

%% Linearized State-Space Matrices


% State vector: [x; x_dot; theta; theta_dot]
% x      = cart position
% x_dot  = cart velocity
% theta  = pendulum angle (rad), 0 = upright
% theta_dot = angular velocity

A = [0 1 0 0; 0 0 (m*g)/M 0; 0 0 0 1; 0 0 ((M+m)*g)/(M*l) 0];

B = [0 1/M 0 1/(M*l)]';
    

% System is of the form: dx/dt = A*x + B*u

%% Set Up LQR Tuning Parameters

% Q penalizes state error: x^T Q x
% Higher values = more priority to keep those states near zero
Q = diag([10, 1, 100, 1]);   % Care a lot about x (cart pos) and theta (angle)

% R penalizes control input effort: u^T R u
% Lower R = more aggro, Higher R = smoother, more conservative
R = 0.01;

%% Compute LQR Gain Matrix

K = lqr(A, B, Q, R);   % Solves Riccati equation and gives optimal K

% Control law: u = -K * x

%%  Closed-Loop System Dynamics (A_cl = A - B*K)


A_cl = A - B * K;   % Closed-loop dynamics
B_cl = B;           % Input matrix (unused in this sim since input = -Kx)
C_cl = eye(4);      % Output all states for plotting
D_cl = zeros(4,1);  % No direct feedthrough

sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

%% Simulate the System

t = linspace(0, 5, 500);  % Simulate for 5 seconds

% Initial condition: small angle (0.1 rad), rest = 0
x0 = [0; 0; 0.5; 0];      

% Simulate response using lsim (zero input, initial state only)
[x, t_out] = lsim(sys_cl, zeros(size(t)), t, x0);

% Compute control input at each time step: u = -K * x
u = -x * K';

%% Plot Results


figure;

subplot(3,1,1);
plot(t, x(:,1), 'LineWidth', 2);
ylabel('Cart Position (m)');
grid on;

subplot(3,1,2);
plot(t, x(:,3), 'LineWidth', 2);
ylabel('Pendulum Angle (rad)');
grid on;

subplot(3,1,3);
plot(t, u, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Control Force (N)');
grid on;

sgtitle('Inverted Pendulum - LQR Control Simulation');

saveas(gcf, '../figures/Inverted_Pendulum_LQR_Sim.png');

% - Cart should return to x = 0
% - Pendulum should stabilize at theta = 0

%%  Simple Animation (Cart + Pendulum)

% Extract simulation data
x_cart = x(:,1);     % cart position
theta  = x(:,3);     % pendulum angle (rad)

L = 1;  % Pendulum visual length 

% Create figure
figure;
axis equal;
axis([-2 2 -1.5 1.5]);
grid on;
title('Inverted Pendulum Animation');
xlabel('Cart Position');
ylabel('Height');

filename = 'pendulum_anim.gif';
% Animation loop
for i = 1:5:length(t)
    clf; hold on;
    
    % Update axis limits dynamically if needed
    axis([-2 2 -1.5 1.5]);
    grid on;
    
    % Cart parameters
    cart_w = 0.3;  % width
    cart_h = 0.2;  % height
    cart_x = x_cart(i) - cart_w/2;
    
    % Draw cart
    rectangle('Position', [cart_x, -0.1, cart_w, cart_h], 'FaceColor', [0.2 0.6 1]);
    
    % Pendulum position
    pend_x = x_cart(i) + L*sin(theta(i));
    pend_y = 0.1 + L*cos(theta(i));
    
    % pendulum rod
    line([x_cart(i), pend_x], [0.1, pend_y], 'LineWidth', 4, 'Color', [0.9 0.2 0.2]);
    
    % pendulum mass
    plot(pend_x, pend_y, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    drawnow;

    frame = getframe(gcf);
im = frame2im(frame);
[imind, cm] = rgb2ind(im, 256);

if i == 1
    imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
else
    imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
end
end


