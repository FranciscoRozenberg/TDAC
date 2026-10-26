clear; close all; clc

%% 1. Cargar datos experimentales del ensayo PD
out_pd = load("PD_0a6_kp2.7_kd0.00002_escalon.mat");
t         = out_pd.out.tout(:);
distancia = out_pd.out.d1(:); % Salida real

%% 2. Definición del Modelo y Controlador PD
p1 = 0;
p2 = -3.5450;
p3 = -6.1249 + 8.5238i;
p4 = -6.1249 - 8.5238i;

% Ganancia de la planta
K = 13.3*38.95;

% Construir la función de transferencia de la planta
s = tf('s');
G = K/((s-p1)*(s-p2)*(s-p3)*(s-p4));

% Parámetros del controlador PD solicitados
Kp = 2.7;
Kd = 0.00002;

% Controlador PD en lazo cerrado
C = Kp + Kd*s;
L = feedback(C * G, 1);

%% 3. Construcción de la entrada de escalón sintética (Magnitud = 6)
t_rel = t - t(1);
u_esc = zeros(size(t_rel));

% Asignamos los escalones de magnitud 6 según los instantes de la prueba
u_esc(t_rel >= 8.5 & t_rel < 16)   = 6; 
u_esc(t_rel >= 24  & t_rel < 32)   = 6; 
u_esc(t_rel >= 40  & t_rel < 48)   = 6;

%% 4. Simulación del modelo con la entrada construida
y_sim = real(lsim(L, u_esc, t_rel));

% Comparar variaciones respecto al valor inicial
y_med     = distancia - distancia(1);
y_sim_var = y_sim - y_sim(1);

%% 5. Gráfica comparativa: Medición real vs Modelo simulado (PD)
figure;
plot(t_rel, y_med, 'b', 'LineWidth', 1.5);
hold on;
plot(t_rel, y_sim_var, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Validación del Modelo con Control PD (Kp = 2.7, Kd = 0.00002)');
legend('Medición real (PD)', 'Modelo simulado (PD)', 'Location', 'best');

%% 6. Cálculo y gráfica del error de predicción
error_pd = y_med - y_sim_var;

figure;
plot(t_rel, error_pd, 'k', 'LineWidth', 1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('Error');
title('Error entre la medición PD y el modelo');