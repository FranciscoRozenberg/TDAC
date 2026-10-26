
clear; close all; clc

inicio = 170;
fin = 800;

out = load("varios_escalones.mat");

angulo_barra = double(out.out.d3(:));
distancia_medida = double(out.out.d4(:));

angulo_barra_sin_retardo = angulo_barra(inicio:fin) - 0.1;
distancia = distancia_medida(inicio:fin) + 0.1;

t = double(out.out.tout(inicio:fin));
Ts = 0.02;

u = angulo_barra_sin_retardo(:);
y = distancia(:);

tiempo = t - t(1);

% Centrar las señales respecto de sus valores iniciales
u_rel = u - u(1);
y_rel = y - y(1);

% Modelo continuo: G(s) = K / (s*(s+a))
s = tf('s');

% Parámetros iniciales para la optimización
K_inicial = 1;
a_inicial = 1;

% Se optimiza K directamente y a en escala logarítmica
parametros_iniciales = [K_inicial, log(a_inicial)];

% Minimizar el error cuadrático entre la medición y la simulación
opciones = optimset('Display','iter', ...
                    'MaxIter',3000, ...
                    'MaxFunEvals',6000, ...
                    'TolX',1e-8, ...
                    'TolFun',1e-8);

parametros = fminsearch( ...
    @(p) error_modelo(p, u_rel, y_rel, tiempo), ...
    parametros_iniciales, opciones);

K =8* parametros(1);
a = exp(parametros(2));

% Planta continua identificada directamente
G_s_id = K/(s*(s+a));

% Simular la respuesta con la entrada experimental
y_modelo_rel = lsim(G_s_id, u_rel, tiempo);

% Volver al nivel original de la medición
y_modelo = y_modelo_rel + y(1);

% Calcular el error cuadrático medio
ECM = mean((y - y_modelo).^2);
RMSE = sqrt(ECM);

% Gráfica de comparación
figure;
plot(tiempo, y, 'k', 'LineWidth', 1.5);
hold on;
plot(tiempo, y_modelo, 'r--', 'LineWidth', 1.5);
grid on;

xlabel('Tiempo [s]');
ylabel('Posición del carrito');
legend('Medición experimental', ...
       'Modelo continuo identificado', ...
       'Location','best');
title('Identificación directa de la planta continua');

% Mostrar resultados
disp('Planta continua identificada:');
G_s_id

disp('Ganancia K:');
disp(K);

disp('Parámetro a:');
disp(a);

disp('Polos de G(s):');
disp(pole(G_s_id));

disp('RMSE:');
disp(RMSE);


% Función de costo
function J = error_modelo(p, u, y, tiempo)

    K = p(1);
    a = exp(p(2));

    s = tf('s');
    G = K/(s*(s+a));

    try
        y_estimado = lsim(G, u, tiempo);
        e = y - y_estimado;

        if any(~isfinite(y_estimado))
            J = 1e100;
        else
            J = mean(e.^2);
        end
    catch
        J = 1e100;
    end
end
