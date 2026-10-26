clear; close all; clc

%% 1. Cargar datos experimentales del impulso
out_imp = load("PI_2.mat");
t         = out_imp.out.tout(:);
distancia = out_imp.out.d1(:); % Salida real

%% 2. Definición del Modelo y Controlador PI
p1 = 0;
p2 = -.8468;
p3 = -8.3056 + 9.4818i;
p4 = -8.3056 - 9.4818i;

K = 4.446*56.28;
s = tf('s');
G = K/((s-p1)*(s-p2)*(s-p3)*(s-p4));

Kp = 2.7;
Ki = 0.22;
C = Kp + Ki/s;
L = feedback(C * G, 1);

%% 3. Simulación analítica de la respuesta al impulso
t_rel = t - t(1);

% Magnitud del impulso teórico (ajusta este factor para escalar la altura del pico a tu gusto)
factor_escala = 1.5; 

% Simulamos la respuesta al impulso del sistema
[y_sim, t_sim] = impulse(L * factor_escala, t_rel(end));

% Desplazamos el vector de tiempo del modelo para que empiece exactamente a los 6 segundos (donde ocurre el ensayo real)
t_impulso_real = 5.2; 
t_sim_desplazado = t_sim + t_impulso_real;

%% 4. Gráfica comparativa alineada
y_med = distancia - distancia(1);

figure;
plot(t_rel, y_med, 'b', 'LineWidth', 1.5);
hold on;
plot(t_sim_desplazado, y_sim, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Respuesta al impulso [cm]');
title('Validación de la Respuesta al Impulso (Control PI)');
legend('Medición real (Impulso)', 'Modelo simulado (Impulso)', 'Location', 'best');