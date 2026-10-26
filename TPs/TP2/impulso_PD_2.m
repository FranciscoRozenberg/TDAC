clear; close all; clc

%% 1. Cargar datos experimentales del ensayo de impulso (PD)
out_imp = load("PD_1.mat");

% Ajusta según el nombre de las variables dentro del archivo .mat (ej: tout y d1)
t         = out_imp.out.tout(:);
distancia = out_imp.out.d1(:); % Salida real medida

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

% Parámetros del controlador PD
Kp = 2.7;
Kd = 0.00002;

% Controlador PD y lazo cerrado
C = Kp + Kd*s;
L = feedback(C * G, 1);

%% 3. Simulación analítica de la respuesta al impulso
t_rel = t - t(1);

% Factor de escala para alinear la altura del pico con la medición real
factor_escala = 1.5; 

% Simular la respuesta al impulso del sistema
[y_sim, t_sim] = impulse(L * factor_escala, t_rel(end));

% Desplazar el vector de tiempo del modelo al instante en que ocurre el impulso en el ensayo real (ej: ~6 segundos)
t_impulso_real = 6.9; 
t_sim_desplazado = t_sim + t_impulso_real;

%% 4. Gráfica comparativa: Impulso Real vs Impulso Simulado (PD)
y_med = distancia - distancia(1);

figure;
plot(t_rel, y_med, 'b', 'LineWidth', 1.5);
hold on;
plot(t_sim_desplazado, y_sim, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Respuesta al impulso [cm]');
title('Validación de la Respuesta al Impulso (Control PD: Kp = 2.7, Kd = 0.00002)');
legend('Medición real (Impulso)', 'Modelo simulado (Impulso)', ...
    'Location', 'best');

%% 5. Cálculo y gráfica del error
% Interpolar la simulación al vector de tiempo experimental para restar punto a punto
y_sim_interp = interp1(t_sim_desplazado, y_sim, t_rel, 'linear', 'extrap');
error_imp = y_med - y_sim_interp;

figure;
plot(t_rel, error_imp, 'k', 'LineWidth', 1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('Error');
title('Error entre la medición del impulso y el modelo PD');