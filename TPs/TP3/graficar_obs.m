%% Script de graficación - Parte 2: Observador de Luenberger
%clear; clc; close all;
clc;
obs = load("mediciones_obs.mat");

t         = double(obs.out.tout(:));
p_med     = double(obs.out.d1(:));
p_est     = double(obs.out.d2(:));
dp_est    = double(obs.out.d3(:));
theta_med = double(obs.out.d4(:));
theta_est = double(obs.out.d5(:));
w_med     = double(obs.out.d6(:));
w_est     = double(obs.out.d7(:));
u_med     = double(obs.out.d8(:));

Ts = .02;

dp_calc = [p_med(1); diff(p_med)] / Ts;

%% Figura 1: Entrada de control u(t)
figure;
stairs(t, u_med, 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Comando u [^\circ]');
xlim([t(1), t(end)]);

%% Figura 2: Posición del carrito p(t)
figure;
plot(t, p_med, 'LineWidth', 1.4, 'DisplayName', 'Medición (HC-SR04)'); hold on;
plot(t, p_est, 'LineWidth', 1.4, 'DisplayName', 'Estimación observador ($\hat{p}$)');
grid on;
xlabel('Tiempo [s]');
ylabel('Posición p [cm]');
legend('Interpreter', 'latex', 'Location', 'best');
xlim([t(1), t(end)]);

%% Figura 3: Ángulo de la barra theta(t)
figure;
plot(t, theta_med, 'LineWidth', 1.4, 'DisplayName', 'Medición (MPU6050)'); hold on;
plot(t, theta_est, 'LineWidth', 1.4, 'DisplayName', 'Estimación observador ($\hat{\theta}$)');
grid on;
xlabel('Tiempo [s]');
ylabel('Ángulo \theta [^\circ]');
legend('Interpreter', 'latex', 'Location', 'best');
xlim([t(1), t(end)]);

%% Figura 4: Velocidad angular dtheta/dt(t)
figure;
plot(t, w_med, 'LineWidth', 1.4, 'DisplayName', 'Medición giróscopo ($\omega$)'); hold on;
plot(t, w_est, 'LineWidth', 1.4, 'DisplayName', 'Estimación observador ($\hat{\dot{\theta}}$)');
grid on;
xlabel('Tiempo [s]');
ylabel('Velocidad angular [^\circ/s]');
legend('Interpreter', 'latex', 'Location', 'best');
xlim([t(1), t(end)]);

%% Figura 5: Velocidad lineal dp/dt(t)
figure;
plot(t, dp_calc, 'LineWidth', 1.2, 'DisplayName', 'Derivada numérica ($\dot{p}_{\mathrm{calc}}$)'); hold on;
plot(t, dp_est, 'LineWidth', 1.5, 'DisplayName', 'Estimación observador ($\hat{\dot{p}}$)');
grid on;
xlabel('Tiempo [s]');
ylabel('Velocidad lineal [cm/s]');
legend('Interpreter', 'latex', 'Location', 'best');
xlim([t(1), t(end)]);