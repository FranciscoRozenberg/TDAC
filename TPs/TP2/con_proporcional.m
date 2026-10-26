clear; close all; clc
out = load("con_proporcional.mat");

distancia = out.out.d1(800:2000);
delta_u   = out.out.d2(800:2000);
t         = out.out.tout(800:2000);

<<<<<<< HEAD
figure;
plot(delta_u);
hold on;
plot(distancia);
grid on;
legend('u = angulo servo', 'y = distancia carrito');
xlabel('Muestra');
ylabel('Ángulo');
title('Ángulo del servo y ángulo de la barra')
=======
% figure;
% plot(delta_u);
% hold on;
% plot(distancia);
% grid on;
% legend('u = angulo servo', 'y = distancia carrito');
% xlabel('Muestra');
% ylabel('Ángulo');
% title('Ángulo del servo y ángulo de la barra')
>>>>>>> e08eb0d5e2b1282a985ab8735e8c8788e10d4899

Ts = 0.02;

u = delta_u(:);
y = distancia(:);
t = t(:);

% Modelo continuo identificado de la planta de 4 polos
p1 = 0;
p2 = -3.3240;
p3 = -6.14589738845825 + 8.52381229400635i;
p4 = -6.14589738845825 - 8.52381229400635i;
% Ganancia de la planta
K = 45.91*38.95;

% Construir la función de transferencia
s = tf('s');
G = K/((s-p1)*(s-p2)*(s-p3)*(s-p4));

% Ver la planta
G
pole(G)

<<<<<<< HEAD
Kp = 0.7;

L = feedback(Kp*G,1)
=======
Kp = 2.7;

L = feedback(Kp*G,1)

u_sym = zeros(1,1201);
u_sym(273) = 150;
u_sym(767) = 150;
>>>>>>> e08eb0d5e2b1282a985ab8735e8c8788e10d4899

% Simular el modelo con la entrada REAL medida
t_rel = t - t(1);
y_sim = real(lsim(L, u_sym, t_rel));

% Comparar variaciones respecto al valor inicial
y_med = y - y(1);
y_sim_var = y_sim - y_sim(1);

%%

<<<<<<< HEAD
u_sym = zeros(1,1201);
u_sym(273) = 150;
u_sym(767) = 150;

% Simular el modelo con la entrada REAL medida
t_rel = t - t(1);
y_sim = real(lsim(L, u_sym, t_rel));

% Comparar variaciones respecto al valor inicial
y_med = y - y(1);
y_sim_var = y_sim - y_sim(1);

% Graficar las dos respuestas
=======
>>>>>>> e08eb0d5e2b1282a985ab8735e8c8788e10d4899
figure;
plot(t_rel, y_med, 'b', 'LineWidth', 1.5);
hold on;
plot(t_rel, y_sim_var, 'r--', 'LineWidth', 1.5);
grid on;

xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Validación experimental del modelo con Control Proporcional');
<<<<<<< HEAD
legend('Medición real', 'Modelo simulado', ...
       'Location', 'best');

% Error de predicción
error = y_med - y_sim_var;

=======
% title('Modelo con Control Proporcional');

legend('Medición real', 'Modelo simulado', ...
    'Location', 'best');

%% Graficar las dos respuestas
figure;
 plot(t_rel, y_med, 'b', 'LineWidth', 1.5);
 hold on;
plot(t_rel, y_sim_var, 'r--', 'LineWidth', 1.5);
grid on;

xlabel('Tiempo [s]');
ylabel('Variación de la salida [cm]');
title('Validación experimental del modelo con Control Proporcional');
% title('Modelo con Control Proporcional');

legend('Medición real', 'Modelo simulado', ...
       'Location', 'best');

%% Error de predicción
error = y_med - y_sim_var;

>>>>>>> e08eb0d5e2b1282a985ab8735e8c8788e10d4899
figure;
plot(t_rel, error, 'k', 'LineWidth', 1.2);
grid on;
xlabel('Tiempo [s]');
ylabel('Error');
title('Error entre la medición y el modelo');
