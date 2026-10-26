clear; close all; clc

%% 1. Cargar datos experimentales del ensayo PI
out_pi = load("PI_0a6_kp2.7ki0.22_escalon.mat");
t         = out_pi.out.tout(:);
distancia = out_pi.out.d1(:); % Salida real

%% 2. Definición del Modelo y Controlador PI
p1 = 0;
p2 = -.8468;
p3 = -8.3056 + 9.4818i;
p4 = -8.3056 - 9.4818i;

% Ganancia de la planta
K = 4.446*56.28;

% Construir la función de transferencia de la planta
s = tf('s');
G = K/((s-p1)*(s-p2)*(s-p3)*(s-p4));

% Parámetros del controlador PI solicitados
Kp = 2.7;
Ki = 0.22;

% Controlador PI en lazo cerrado
C = Kp + Ki/s;
L = feedback(C * G, 1);

%% 3. Construcción de la entrada de escalón sintética (Magnitud = 5)
t_rel = t - t(1);
u_esc = zeros(size(t_rel));

% Asignamos los escalones de magnitud 5 según los instantes de la prueba
% (Ajusta los rangos de tiempo si el escalón arranca en otro segundo exacto)
u_esc(t_rel >= 8.5 & t_rel < 16)   = 5; 
u_esc(t_rel >= 24  & t_rel < 32)   = 5; 
u_esc(t_rel >= 40  & t_rel < 48)   = 5;

%% 4. Simulación del modelo con la entrada real simulada
y_sim = real(lsim(L, u_esc, t_rel));

% Comparar variaciones respecto al valor inicial
y_med     = distancia - distancia(1);
y_sim_var = y_sim - y_sim(1);

%% 5. Gráfica comparativa: Medición real vs Modelo simulado (PI)
figure;
plot(t_rel, y_med, 'b', 'LineWidth', 1.5);
hold on;
plot(t_rel, y_sim_var, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Validación del Modelo con Control PI (Kp = 2.7, Ki = 0.22)');
legend('Medición real (PI)', 'Modelo simulado (PI)', 'Location', 'best');

%% 6. Cálculo y gráfica del error de predicción
error_pi = y_med - y_sim_var;

figure;
plot(t_rel, error_pi, 'k', 'LineWidth', 1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('Error');
title('Error entre la medición PI y el modelo');