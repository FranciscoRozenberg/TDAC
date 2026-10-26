%save('alpha.mat') % ejecutar una sola vez
clear; close all; clc
format long g
out = load("alpha.mat");

u_real = out.out.d5(4050:4650);
y_real = out.out.d3(4050:4650)-0.005;

Ts = 0.02;

u = u_real(:);
y = y_real(:);

N = length(y);
t = (0:N-1)'*Ts;

%% Identificacion por cuadrados minimos
% y[k] = -a1*y[k-1] - a2*y[k-2]+ b1*u[k-1] + b2*u[k-2]

Phi = [-y(2:N-1),-y(1:N-2),u(2:N-1),u(1:N-2)];

Y = y(3:N);

theta = pinv(Phi)*Y;

a1 = theta(1);
a2 = theta(2);
b1 = theta(3);
b2 = theta(4);

fprintf('Coeficientes identificados:\n');
fprintf('a1 = %.8f\n', a1);
fprintf('a2 = %.8f\n', a2);
fprintf('b1 = %.8f\n', b1);
fprintf('b2 = %.8f\n', b2);

%% Transferencia discreta
G_z = tf([b1 b2], [1 a1 a2], Ts);
disp('Modelo discreto G(z):')
G_z

%% Recuperacion aproximada del modelo continuo sin ceros
K_dc = (b1+b2)/(1+a1+a2);

polos_z = roots([1 a1 a2]);
polos_s = log(polos_z)/Ts;

den_s = real(poly(polos_s));
num_s = K_dc*den_s(3);

G_s = tf(num_s, den_s);

disp('Polos discretos:')
disp(polos_z)

disp('Polos continuos:')
disp(polos_s)

disp('Modelo continuo aproximado G(s):')
G_s

%% Simulacion con la entrada real
% El modelo identificado tiene un retardo de una muestra.

u_ret = [0; u(1:end-1)];

y_sim_z = filter([b1 b2], [1 a1 a2], u_ret);

% Simulacion de la planta continua con entrada mantenida
% constante entre muestras (ZOH).
y_sim_s = lsim(G_s, u, t, 'zoh');

%% Comparacion
figure('Color','w');
plot(t, y, 'b', 'LineWidth', 1.2);
hold on;
plot(t, y_sim_z, 'r--', 'LineWidth', 1.5);
plot(t, y_sim_s, 'k:', 'LineWidth', 1.5);

grid on;
xlabel('Tiempo [s]');
ylabel('Amplitud');
title('Identificacion usando la entrada real');
legend('Salida experimental', ...
       'Modelo discreto G(z)', ...
       'Modelo continuo G(s)', ...
       'Location','best');

%% Comprobacion de la entrada utilizada
figure('Color','w');

subplot(2,1,1)
plot(t, u, 'LineWidth', 1.2);
grid on;
ylabel('u[k]');
title('Entrada real aplicada');

subplot(2,1,2)
plot(t, y, 'LineWidth', 1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('y[k]');
title('Salida medida');