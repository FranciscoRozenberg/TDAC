clear; close all; clc

%% 1. Cargar datos experimentales
out_esc = load("P_0a6_kp2.7_escalon.mat");
t         = out_esc.out.tout(:);
distancia = out_esc.out.d1(:);

%% 2. Definición del Modelo y Controlador
p1 = 0;
p2 = -.8468;
p3 = -8.3056 + 9.4818i;
p4 = -8.3056 - 9.4818i;

% Ganancia de la planta
K = 4.446*56.28;

% Construir la función de transferencia
s = tf('s');
G = K/((s-p1)*(s-p2)*(s-p3)*(s-p4));

Kp = 2.7;
L = feedback(Kp*G, 1);

%% 3. Construcción de la entrada de escalón sintética (con los tiempos y magnitud correctos)
t_rel = t - t(1);
u_esc = zeros(size(t_rel));

% Basado en tu gráfica anterior, los escalones ocurren aproximadamente en:
% - Primer escalón (sube a 5): ~8.5s a 16s
% - Segundo escalón (sube a ~12 / baja): ~24s a 32s
% - Tercer escalón: ~40s a 48s
% Puedes ajustar estos índices o tiempos exactos según tu vector de tiempo 't_rel':

% Ejemplo detectando los instantes mediante condiciones lógicas sobre el tiempo o muestras:
% (O puedes asignarlo por rangos de índices si prefieres, ej: t_rel >= 8.5 & t_rel < 16)
u_esc(t_rel >= 8.5 & t_rel < 16)   = 5; 
u_esc(t_rel >= 24  & t_rel < 32)   = 5; % Ajusta la altura si el segundo escalón fue distinto
u_esc(t_rel >= 40  & t_rel < 48)   = 5;

%% 4. Simulación del modelo con la entrada construida
y_sim = real(lsim(L, u_esc, t_rel));

% Comparar variaciones respecto al valor inicial
y_med     = distancia - distancia(1);
y_sim_var = y_sim - y_sim(1);

%% 5. Graficar la comparación
figure;
plot(t_rel, y_med, 'b', 'LineWidth', 1.5);
hold on;
plot(t_rel, y_sim_var, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Validación del Modelo ante Tren de Escalones (Magnitud = 5)');
legend('Medición real', 'Modelo simulado', 'Location', 'best');

%% 6. Cálculo y gráfica del error
error = y_med - y_sim_var;

figure;
plot(t_rel, error, 'k', 'LineWidth', 1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('Error');
title('Error entre la medición y el modelo simulado');