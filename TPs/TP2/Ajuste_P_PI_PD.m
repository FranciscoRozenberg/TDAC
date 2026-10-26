%% Identificacion de controladores P, PI y PD
% Para planta ByB

%%

% Modelo continuo identificado de la planta de 4 polos
clear; close all; clc
p1 = 0;
p2 = -3.5450;
p3 = -6.1249 + 8.5238i;
p4 = -6.1249 - 8.5238i;

% Ganancia de la planta
K = 13.3*38.95;

% Construir la función de transferencia
s = tf('s');
G = K/((s-p1)*(s-p2)*(s-p3)*(s-p4));

% Ver la planta
G
p = pole(G);

disp(" ")
disp("Polos de G(s): " + p(1) +", " + p(2) +", " + p(3) +", " + p(4))

%% Respuesta simulada para distintos Kp
figure;
KpValues = [0.5 1 2 3 4 5 6.15];
t_final = 40; 
Ts = 0.02;    
t = 0:Ts:t_final;

for index = 1:length(KpValues)
    L = feedback(KpValues(index)*G,1);
    y_sym= impulse(L, t);
    plot(t, real(y_sym), 'LineWidth', 2);
    hold on;
end
grid on;
legend("K_p = " + string(KpValues), 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Modelo con Control Proporcional');
hold off

%% Control P
% Por analisis empirico: $K_0$ = 6,15 y $T_0$ = 1,26. Ajuste inicial por Ziegler-Nichols

k0 = 6.15;
T0 = 1.26;

Kp = .5*k0;

disp("Kp: " + num2str(Kp))

L = feedback(Kp*G, 1)

figure;
impulse(L, t);
grid on;
title("Respuesta al escalón con control proporcional");
xlabel("Tiempo [s]");
ylabel("Salida [cm]");

%%
% Respuesta oscila. Se busca Coeficiente maximo sin oscilar.
KpValues = [.7 .8 .9 .95];
t_final = 40; 
Ts = 0.02;    
t = 0:Ts:t_final;

% Impulso
figure;
for index = 1:length(KpValues)
    L = feedback(KpValues(index)*G,1);
    y_sym= impulse(L, t);
    plot(t, real(y_sym), 'LineWidth', 2);
    hold on;
end
grid on;
legend("K_p = " + string(KpValues), 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Modelo con Control Proporcional (impulso)');
hold off

% Escalon
figure;
for index = 1:length(KpValues)
    L = feedback(KpValues(index)*G,1);
    opt = stepDataOptions('StepAmplitude', 10);
    y_sym= step(L, t, opt);
    plot(t, real(y_sym), 'LineWidth', 2);
    hold on;
end
grid on;
legend("K_p = " + string(KpValues), 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Modelo con Control Proporcional (escalon)');
hold off

Kp = .8;

disp("Kp maximo sin oscilar: " + Kp)


%% Control PI
% 

s = tf("s");

KiValues = [.05 .1 .15 .2 .25];

% Impulso
figure;
for index = 1:length(KiValues)
    L = feedback( (KiValues(index)/s + Kp) *G ,1);
    y_sym= impulse(L, t);
    plot(t, real(y_sym), 'LineWidth', 2);
    hold on;
end
grid on;
legend("K_i = " + string(KiValues), 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Modelo con Control Proporcional Integral (impulso)');
hold off

% Escalon
figure;
for index = 1:length(KiValues)
    L = feedback( (KiValues(index)/s + Kp) *G,1);
    opt = stepDataOptions('StepAmplitude', 10);
    y_sym= step(L, t, opt);
    plot(t, real(y_sym), 'LineWidth', 2);
    hold on;
end
grid on;
legend("K_i = " + string(KiValues), 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Modelo con Control Proporcional Integral (escalon)');
hold off

Ki = .15;

% Criterio: buen equilibrio entre sobrepico y velocidad de estabilizacion
disp("Ki: " + Ki)

%% Control PD
% 

s = tf("s");

KdValues = [.5e-3 .10e-3 .5e-2 .10e-2 .5 1];

% Impulso
figure;
for index = 1:length(KdValues)
    L = feedback( (KdValues(index)*s + Kp) *G ,1);
    y_sym= impulse(L, t);
    plot(t, real(y_sym), 'LineWidth', 2);
    hold on;
end
grid on;
legend("K_d = " + string(KdValues), 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Modelo con Control Proporcional Derivativo (impulso)');
hold off

% Escalon
figure;
for index = 1:length(KdValues)
    L = feedback( (KdValues(index)*s + Kp) *G,1);
    opt = stepDataOptions('StepAmplitude', 10);
    y_sym= step(L, t, opt);
    plot(t, real(y_sym), 'LineWidth', 2);
    hold on;
end
grid on;
legend("K_d = " + string(KdValues), 'Location', 'best');
xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Modelo con Control Proporcional Derivativo (escalon)');
hold off

Kd = .5;

disp("Kd maximo sin oscilacion excesiva: " + Kd)

% %%
% 
% figure;
% plot(t_rel, y_med, 'b', 'LineWidth', 1.5);
% hold on;
% plot(t_rel, y_sim_var, 'r--', 'LineWidth', 1.5);
% grid on;
% 
% xlabel('Tiempo [s]');
% ylabel('Variación de la salida [cm]');
% title('Validación experimental del modelo con Control Proporcional');
% % title('Modelo con Control Proporcional');
% 
% legend('Medición real', 'Modelo simulado', ...
%     'Location', 'best');
% 
% %% Graficar las dos respuestas
% figure;
%  plot(t_rel, y_med, 'b', 'LineWidth', 1.5);
%  hold on;
% plot(t_rel, y_sim_var, 'r--', 'LineWidth', 1.5);
% grid on;
% 
% xlabel('Tiempo [s]');
% ylabel('Variación de la salida [cm]');
% title('Validación experimental del modelo con Control Proporcional');
% % title('Modelo con Control Proporcional');
% 
% legend('Medición real', 'Modelo simulado', ...
%        'Location', 'best');
% 
% %% Error de predicción
% error = y_med - y_sim_var;
% 
% figure;
% plot(t_rel, error, 'k', 'LineWidth', 1.2);
% grid on;
% xlabel('Tiempo [s]');
% ylabel('Error');
% title('Error entre la medición y el modelo');
