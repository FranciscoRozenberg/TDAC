%save('alpha.mat') % ejecutar una sola vez
clear; close all; clc
inicio=170;
fin=400;
out = load("varios_escalones.mat");
angulo_barra = out.out.d3(:);

angulo_barra_sin_retardo = double(angulo_barra(inicio:fin))-0.1;
distancia = double(out.out.d4(inicio:fin))+0.1;

retardo=0;
angulo_barra_con_retardo = angulo_barra(inicio-retardo:fin-retardo);
t = out.out.tout(inicio:fin);
Ts = 0.02;
u = angulo_barra_con_retardo(:);
y = distancia(:);

figure;
plot(angulo_barra_sin_retardo);
hold on;
plot(distancia);
grid on;
legend('u = angulo de la barra', 'y = distancia del carrito');
xlabel('Muestra');
ylabel('Ángulo');
title('Entrada y salida sitema barra-carrito');

N = length(y);

Phi = [-(y(2:N-1) + y(1:N-2)), u(1:N-2)];
Y = y(3:N)+ y(2:N-1);
theta = pinv(Phi)*Y;
a = theta(1);
k = theta(2);

% Planta discreta identificada por regresion
G_z_id = tf([k], [1 1+a a], Ts);

polos_z = roots([1 1+a a]);
polos_s = log(polos_z) / Ts;

p1 = 0;
p2 = -6*polos_s(1);

ganancia_estatica =(k)/(1 +1+a+ a);
K_cont = 1*p2* ganancia_estatica;
% Retardo de una muestra: 20 ms
[num1, den1] = pade(Ts, 1);
G_pade1 = tf(num1, den1);
% Retardo de dos muestras: 40 ms
[num2, den2] = pade(2*Ts, 1);
G_pade2 = tf(num2, den2);
% Retardo de dos muestras: 60 ms
[num3, den3] = pade(3*Ts, 1);
G_pade3 = tf(num3, den3);

%PLANTA CONTINUA
s = tf('s');
G_s_id = (K_cont/((s - p1)*(s - p2))) ;

% Respuesta de ambas plantas ante la misma entrada
tiempo = t - t(1);

y_discreta = lsim(G_z_id, u, tiempo);
y_continua = lsim(G_s_id, u, tiempo);

% Grafica
figure;
plot(tiempo, y, 'k', 'LineWidth', 1.5);
hold on;
plot(tiempo, y_discreta, 'b--', 'LineWidth', 1.5);
plot(tiempo, y_continua, 'r-.', 'LineWidth', 1.5);

grid on;
xlabel('Tiempo [s]');
ylabel('Ángulo de la barra');
legend('Medición experimental','Planta discreta identificada','Planta continua ajustada','Location', 'best');
title('Comparación de respuestas');

% Mostrar resultados
disp('Coeficientes de la regresion:');
disp(theta);

disp('Planta discreta identificada:');
G_z_id

disp('Planta continua ajustada:');
G_s_id

polos_Gs = pole(G_s_id);
disp('Polos de G(s):')
disp(polos_Gs)