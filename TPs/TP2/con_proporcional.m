clear; close all; clc
out = load("con_proporcional.mat");

distancia = out.out.d1(800:1300);
delta_u   = out.out.d3(800:1300);
t         = out.out.tout(800:1300);

figure;
plot(delta_u);
hold on;
plot(distancia);
grid on;
legend('u = angulo servo', 'y = angulo barra');
xlabel('Muestra');
ylabel('Ángulo');
title('Ángulo del servo y ángulo de la barra')

Ts = 0.02;

u = delta_u(:);
y = distancia(:);
t = t(:);

% Modelo continuo identificado de la planta de 4 polos
p1 = ...;
p2 = ...;
p3 = ...;
p4 = ...;

% Ganancia de la planta
K = ...;

% Construir la función de transferencia
s = tf('s');
G = K/((s-p1)*(s-p2)*(s-p3)*(s-p4));

% Ver la planta
G
pole(G)

% Simular el modelo con la entrada REAL medida
t_rel = t - t(1);
y_sim = lsim(G, u, t_rel);

% Comparar variaciones respecto al valor inicial
y_med = y - y(1);
y_sim_var = y_sim - y_sim(1);

% Graficar las dos respuestas
figure;
plot(t_rel, y_med, 'b', 'LineWidth', 1.5);
hold on;
plot(t_rel, y_sim_var, 'r--', 'LineWidth', 1.5);
grid on;

xlabel('Tiempo [s]');
ylabel('Variación de la salida');
title('Validación experimental de la planta');
legend('Medición real', 'Modelo simulado', ...
       'Location', 'best');

% Error de predicción
error = y_med - y_sim_var;

figure;
plot(t_rel, error, 'k', 'LineWidth', 1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('Error');
title('Error entre la medición y el modelo');
