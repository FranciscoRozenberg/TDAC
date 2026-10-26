clear; close all; clc
% out = load("con_proporcional.mat");
% 
% distancia = out.out.d1(800:2000);
% delta_u   = out.out.d2(800:2000);
% t         = out.out.tout(800:2000);
% 
% figure;
% plot(delta_u);
% hold on;
% plot(distancia);
% grid on;
% legend('u = angulo servo', 'y = distancia carrito');
% xlabel('Muestra');
% ylabel('Ángulo');
% title('Ángulo del servo y ángulo de la barra')
% 
% Ts = 0.02;
% 
% u = delta_u(:);
% y = distancia(:);
% t = t(:);

% Modelo continuo identificado de la planta de 4 polos
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
pole(G)

% Kp = 2.7;
% 
% L = feedback(Kp*G,1)
% 
% u_sym = zeros(1,1201);
% u_sym(273) = 150;
% u_sym(767) = 150;

% % Simular el modelo con la entrada REAL medida
% t_rel = t - t(1);
% y_sim = real(lsim(L, u_sym, t_rel));

% % Comparar variaciones respecto al valor inicial
% y_med = y - y(1);
% y_sim_var = y_sim - y_sim(1);


%% Respuesta simulada para distintos Kp
figure;
KpValues = [0.5 1 2 3 4 5 6.15];
t_final = 40; % Ajusta este valor (en segundos) según la lentitud de tu planta G
Ts = 0.02;    % Paso de tiempo para la simulación
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

%% Estabilizar la Planta
% Por analisis empirico: $K_0$ = 6,15 y $T_0$ = 1,26


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
