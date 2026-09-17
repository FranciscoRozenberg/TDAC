out=load("alpha.mat");
alphak = out.out.d1;
alphaa = out.out.d2;
alpha = out.out.d3;

%stairs(alpha)

y = alpha(4500:6500);

%% 1. Configuración de parámetros
Ts = 0.02;                 % Tiempo de muestreo: 20 ms
A_step = 0.184;            % Amplitud del escalón de entrada u(k)
N_transitorio = 30;        % Muestras para la regresión (transitorio dinámico)
N_plot_estacionario = 60;   % Muestras para la visualización extendida
min_dist = 50;             % Distancia mínima entre flancos sucesivos

y = y(:);

%% 2. Detección de flancos (Cruces por cero)
% Flancos ascendentes
cruces_up = find(y(1:end-1) <= 0 & y(2:end) > 0) + 1;
idx_validos = [true; diff(cruces_up) > min_dist];
flancos_up = cruces_up(idx_validos);

% Flancos descendentes
cruces_down = find(y(1:end-1) >= 0 & y(2:end) < 0) + 1;

% Descartar ciclos incompletos
flancos_up = flancos_up(flancos_up + N_transitorio <= length(y));
num_ciclos = length(flancos_up);

%% 3. Construcción matricial apilada con u(k) sintetizada
Phi_total = [];
Y_total   = [];

for i = 1:num_ciclos
    k0 = flancos_up(i);
    rango = k0:(k0 + N_transitorio);
    y_win = y(rango);
    
    % Filtrar posibles trazas espurias planas
    if max(y_win) < (A_step * 0.4)
        continue;
    end
    
    M = length(y_win);
    
    % u[k] es estrictamente la serie de unos multiplicada por el escalón
    u_win = ones(M, 1) * A_step;
    
    % Regresores: y[k] = -a1*y[k-1] - a2*y[k-2] + b1*u[k-1] + b2*u[k-2]
    Phi_i = [ -y_win(2:M-1),  -y_win(1:M-2),  u_win(2:M-1),  u_win(1:M-2) ];
    Y_i   = y_win(3:M);
    
    Phi_total = [Phi_total; Phi_i];
    Y_total   = [Y_total; Y_i];
end

%% 4. Regresión ECM y Duplicación de Ganancia DC
theta_ecm = pinv(Phi_total) * Y_total;

a1 = theta_ecm(1);
a2 = theta_ecm(2);
b1 = theta_ecm(3);
b2 = theta_ecm(4);

% Duplicación de la ganancia estática escalando el numerador

fprintf('--- Coeficientes Discretos Identificados (Ganancia x2) ---\n');
fprintf('a1 = %.6f\na2 = %.6f\nb1 = %.6f\nb2 = %.6f\n\n', a1, a2, b1, b2);

%% 5. Transferencias G(z) y G(s) sin ceros

% Modelo discreto identificado
num_d = [b1, b2];
den_d = [1, a1, a2];
G_z = tf(num_d, den_d, Ts);

% Ganancia estática
K_dc = (b1 + b2) / (1 + a1 + a2);

% Mapeo de polos a tiempo continuo: s = ln(z) / Ts
polos_z = roots(den_d);
polos_s = log(polos_z) / Ts;

% Polinomio continuo estricto de 2do orden sin ceros
den_s = real(poly(polos_s));  % [1, 2*zeta*wn, wn^2]
num_s = K_dc * den_s(3);      % Preserva exactamente la ganancia estática K_dc

G_s = tf(num_s, den_s);

wn = sqrt(den_s(3));
zeta = den_s(2) / (2 * wn);

fprintf('--- Modelo Continuo G(s) sin ceros ---\n');
fprintf('Frecuencia natural (wn)  : %.3f rad/s\n', wn);
fprintf('Amortiguamiento (zeta)   : %.4f\n', zeta);
fprintf('Ganancia estática (K_dc) : %.4f\n\n', K_dc);
G_s

G_z

sign_a1 = '+'; if -a1 < 0, sign_a1 = '-'; end
sign_a2 = '+'; if -a2 < 0, sign_a2 = '-'; end
sign_b2 = '+'; if  b2 < 0, sign_b2 = '-'; end

fprintf('   y[k] = %s %.6f y[k-1] %s %.6f y[k-2] + %.6f u[k-1] %s %.6f u[k-2]\n\n', ...
    sign_a1, abs(a1), sign_a2, abs(a2), b1, sign_b2, abs(b2));

%% 6. Validación Visual 1: Transitorio y Estacionario Multitraza
figure('Color', 'w', 'Name', 'Validación Multitraza'); 
hold on; grid on;

t_plot = (0:N_plot_estacionario)' * Ts;
% Entrada para la simulación: serie de unos por la amplitud del escalón
u_plot = ones(size(t_plot)) * A_step;

% Graficar ciclos experimentales
for i = 1:num_ciclos
    k0 = flancos_up(i);
    next_down = cruces_down(cruces_down > k0);
    if isempty(next_down)
        k_max_ciclo = length(y);
    else
        k_max_ciclo = next_down(1) - 1;
    end
    
    len_disp = min(N_plot_estacionario + 1, k_max_ciclo - k0 + 1);
    t_ciclo = (0:len_disp-1)' * Ts;
    y_ciclo = y(k0 : k0 + len_disp - 1);
    
    if max(y_ciclo) < (A_step * 0.4), continue; end
    
    if i == 1
        plot(t_ciclo, y_ciclo, 'Color', [0.75 0.75 0.75], 'LineWidth', 1.0, ...
             'DisplayName', sprintf('Ciclos medidos (N=%d)', num_ciclos));
    else
        plot(t_ciclo, y_ciclo, 'Color', [0.75 0.75 0.75], 'LineWidth', 1.0, ...
             'HandleVisibility', 'off');
    end
end

xline(N_transitorio * Ts, 'k--', 'LineWidth', 1.2, ...
      'DisplayName', sprintf('Fin de ventana de ajuste (N=%d)', N_transitorio));

% Simulación de la respuesta ante la entrada u_plot
y_sim_ext = filter(num_d, den_d, [0; u_plot(1:end-1)]);
[y_sim_s, t_out_s] = step(G_s * A_step, t_plot(end));

plot(t_plot, y_sim_ext, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Modelo G(z)');
stairs(t_out_s, y_sim_s, 'b--', 'LineWidth', 1.8, 'DisplayName', 'Modelo G(s)');
%plot(t_plot, u_plot, 'g:', 'LineWidth', 1.5, 'DisplayName', 'Entrada u(t) = A_{step}');

xlabel('Tiempo relativo [s]');
ylabel('Amplitud');
title('Respuesta al Escalón: Modelo vs Mediciones (Ganancia x2)');
legend('Location', 'SouthEast');

%% 7. Validación Visual 2: Serie Temporal Cuadrada Completa
figure('Color', 'w', 'Name', 'Validación Señal Cuadrada Completa');
hold on; grid on;

N_total = length(y);
t_total = (0:N_total-1)' * Ts;

% Reconstrucción de la señal cuadrada sintética completa
u_cuadrada = zeros(N_total, 1);
for i = 1:num_ciclos
    k_start = flancos_up(i);
    next_down = cruces_down(cruces_down > k_start);
    if isempty(next_down)
        k_end = N_total;
    else
        k_end = next_down(1) - 1;
    end
    u_cuadrada(k_start:k_end) = A_step;
end

% Simulación con la entrada cuadrada global
y_sim_cuadrada = filter(num_d, den_d, [0; u_cuadrada(1:end-1)]);

stairs(t_total, y, 'Color', [0.2 0.5 0.9], 'LineWidth', 1.2, 'DisplayName', 'Salida Medida y[k]');
%plot(t_total, u_cuadrada, 'Color', [0.3 0.8 0.3], 'LineStyle', ':', 'LineWidth', 1.5, 'DisplayName', 'Entrada Cuadrada u[k]');
plot(t_total, y_sim_cuadrada, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Modelo G(z)');

xlabel('Tiempo [s]');
ylabel('Amplitud');
title('Serie Completa: Medición Real vs Simulación G(z)');
legend('Location', 'NorthEast');