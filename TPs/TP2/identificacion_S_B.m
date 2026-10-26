%save('alpha.mat') % ejecutar una sola vez
clear; close all; clc
out = load("Escalon_servo_barra.mat");
retardo=1;
angulo_servo = out.out.d5(4050:4650);
angulo_barra = (out.out.d3(4050+retardo:4650+retardo) - 0.0037)*180/pi;
t = out.out.tout(4050:4650);
Ts = 0.02;

u = angulo_servo(:);
y = angulo_barra(:);

tiempo = t - t(1);
figure;
plot(tiempo, u, 'b', 'LineWidth', 1.2);
hold on;
plot(tiempo, y, 'r', 'LineWidth', 1.2);

grid on;
legend('Ángulo del servo', 'Ángulo de la barra');
xlabel('Tiempo [s]');
ylabel('Ángulo');
title('Ángulo del servo y de la barra');

syms s K a b z

% Planta continua
G_s = K/(s^2 + a*s + b);

% Sustitución de Tustin
G_z = simplify(subs(G_s, s, (2/Ts)*(z-1)/(z+1)));

[num, den] = numden(G_z);

num = collect(expand(num), z);
den = collect(expand(den), z);

disp('Numerador:')
pretty(num)

disp('Denominador:')
pretty(den)

N = length(y);

Phi = [-y(2:N-1), -y(1:N-2), u(3:N)];
Y = y(3:N);
theta = pinv(Phi)*Y;
a1 = theta(1);
a2 = theta(2);
b0 = theta(3);

% Planta discreta identificada por regresion
G_z_id = tf([b0], [1 a1 a2], Ts);

polos_z = roots([1 a1 a2]);
polos_s = log(polos_z) / Ts;

p1 = 1*(polos_s(1))+0i;
p2 = 1*(polos_s(2))-0i;

ganancia_estatica = (b0 )/(1 + a1 + a2);
K_cont = 1*p1*p2* ganancia_estatica;
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
legend('Medición experimental','Planta discreta identificada','Planta continua equivalente','Location', 'best');
title('Comparación de la respuesta experimental y los modelos identificados con retardo');

% Mostrar resultados
disp('Coeficientes de la regresion:');
disp(theta);

disp('Planta discreta identificada:');
G_z_id

disp('Planta continua equivalente:');
G_s_id

polos_Gs = pole(G_s_id);
disp('Polos de G(s):')
disp(polos_Gs)