%% Practica 7
clear; clc; 
%%
Ts = 1/50; % [ seg ]

num = 56.28;
den = [1 ,16.61 ,158.9];

P = tf(num,den);

Pd = c2d(P,Ts,'zoh');

pole(P)
pole(Pd)

%[A,B,C,D] = tf2ss(num,den);

A = [   0    1   0  ; 
     -158  -16.61 0 ;
     0      0     1];
B = [ 0    ; 
     56.28 ;
     0];
C = [1  0  0 ;
     0  1  1];
D = 0;

sysc = ss(A, B, C, D)

sysd = c2d(sysc, Ts, "zoh");

%%
% Matrices de espacio de estados:
disp("A:")
pretty(vpa(sym(A),4))

disp("B:")
pretty(vpa(sym(B),4))

disp("C:")
pretty(vpa(sym(C),4))

disp("D:")
pretty(vpa(sym(D),4))
%%
O = [A;C];
if rank(O) == 2
    disp("Sistema es Observable")
else
    disp("Sistema NO es Observable")
end

%%
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;
%% 
p_cont = [-20 + 19i, -20 - 19i,-20];
p_obs  = exp(p_cont * Ts);
p_obs__=[0.6225 + 0.2486i  , 0.6225 - 0.2486i  , 0.6703];
% Cálculo de ganancia L
L = place(Ad', Cd', p_obs__)';

% Polos del observador: autovalores de (Ad - L*Cd)
pol = eig(Ad - L*Cd);
%pol = eig(Ad - Cd*L);

disp("Ad:")
pretty(vpa(sym(Ad),4))

disp("Bd:")
pretty(vpa(sym(Bd),4))

disp("Cd:")
pretty(vpa(sym(Cd),4))

disp("L:")
pretty(vpa(sym(L),4))

disp("Polos del Controlador:")
pretty(vpa(sym(pol),4))


%% Sistema Virtual

[yd, td] = step(sysd);

[yc, tc] = step(sysc);

y_r = yc + .01*randn(length(yc),1);

%%
figure();
plot(tc, y_r, LineWidth=3);
hold on
plot(tc, yc)
hold off

%% Simulación recursiva del Observador de Estados Discreto
% Se utiliza un bucle explícito para iterar las ecuaciones de estado

% 1. Definición del vector de tiempo discreto basado en el escalón
t_sim = 0:Ts:5;                % Simulación de 0 a 5 segundos
N = length(t_sim);
u = ones(1, N);                % Entrada escalón unitario

% 2. Generación del ruido de medición
sigma_v = 0.01;
v = sigma_v * randn(1, N);     % Ruido blanco gaussiano

% 3. Inicialización de vectores de estado y salidas
x_real = zeros(2, N);          % Estado real: [ángulo; velocidad]
x_hat  = zeros(2, N);          % Estado estimado por el observador
y_real = zeros(1, N);          % Salida real (sin ruido)
y_r    = zeros(1, N);          % Salida medida (con ruido)
y_hat  = zeros(1, N);          % Salida estimada

% 4. Bucle explícito de simulación (x_n+1 = A*x_n + ...)
for n = 1:N-1
    % --- Salida y Medición en el instante n ---
    y_real(n) = Cd * x_real(:, n);
    y_r(n)    = y_real(n) + v(n);
    y_hat(n)   = Cd * x_hat(:, n);
    
    % --- Ecuación de Estado de la Planta Real ---
    % x_real(n+1) = Ad * x_real(n) + Bd * u(n)
    x_real(:, n+1) = Ad * x_real(:, n) + Bd * u(n);
    
    % --- Ecuación de Estado del Observador Discreto ---
    % x_hat(n+1) = Ad * x_hat(n) + Bd * u(n) + L * ( y_r(n) - Cd * x_hat(n) )
    x_hat(:, n+1) = Ad * x_hat(:, n) + Bd * u(n) + L * (y_r(n) - y_hat(n));
end

% Cálculo del último punto de salida
y_real(N) = Cd * x_real(:, N);
y_r(N)    = y_real(N) + v(N);
y_hat(N)  = Cd * x_hat(:, N);

%% Gráficos comparativos
figure();

% Subplot 1: Medición ruidosa vs Estimación del ángulo (x1)
plot(t_sim, y_r, 'x', 'MarkerSize', 10, 'DisplayName', 'Medición ruidosa (y_r)'); hold on;
stairs(t_sim, y_hat, '-', 'LineWidth', 1.8, 'DisplayName', 'Ángulo estimado (\hat{\theta})');
stairs(t_sim, y_real, '--', 'LineWidth', 1.2, 'DisplayName', 'Ángulo real (\theta)');

grid on;
ylabel('Ángulo [rad]');
title('Observador de Estados Discreto (Iteración Muestra a Muestra)');
legend('Location', 'southeast');

figure();
% Subplot 2: Reconstrucción de la Velocidad Angular (x2 - Estado no medido)
stairs(t_sim, x_hat(2, :), '-', 'LineWidth', 1.8, 'DisplayName', 'Velocidad estimada (\hat{\omega})');
stairs(t_sim, x_real(2, :), '--', 'LineWidth', 1.2, 'DisplayName', 'Velocidad real (\omega)'); hold on;

grid on;
xlabel('Tiempo [s]');
ylabel('Velocidad Angular [rad/s]');
legend('Location', 'southeast');
%%
vars = load("/home/CuadradoGris/Documents/TdAC/practica_7_observadores/variables.mat");

omega_med = vars.out.d1  ;
theta_med = vars.out.d2  ;
theta_obs = vars.out.d3  ;
omega_obs = vars.out.d4  ;
t_med     = vars.out.tout;

clear; clc; close all;

Ts = 1/50; % Periodo de muestreo [s] (50 Hz)

%% 1. Modelo Continuo y Discreto Aumentado (3 estados)
% Estados: x1 = theta [deg], x2 = omega [deg/s], x3 = beta (bias) [deg/s]
A_aug = [   0      1    0 ;
         -158.9 -16.61  0 ;
            0      0    0 ];

B_aug = [ 0 ; 56.28 ; 0 ];
C_aug = [1, 0, 0];
D_aug = 0;

sys_c = ss(A_aug, B_aug, C_aug, D_aug);
sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;

%% 2. Diseño del Observador (Asignación de polos)
% Polos continuos: 2 dinámicos del sistema (-20 +- 19i) y 1 lento para el bias (-3)
p_cont = [-20 + 19i, -20 - 19i, -3];
p_obs  = exp(p_cont * Ts); % Mapeo a plano Z

% Cálculo de ganancia L (3x1) por dualidad
L = place(Ad', Cd', p_obs)';

disp('Ganancia L calculada:');
disp(L);

%% 3. Simulación con Sesgo Inyectado
N = 300;                      % 6 segundos de simulación (300 muestras)
t = (0:N-1) * Ts;
u = zeros(1, N);              % Entrada u = 0 (barra quieta para ver el sesgo puro)

% Inyección de Sesgo Real en la planta
beta_real = 3.5;              % Sesgo constante de +3.5 deg/s en el giróscopo
sigma_ruido = 0.05;           % Ruido gaussiano de medición

% Reservar memoria para vectores
x_real = zeros(3, N);
x_hat  = zeros(3, N);
y_med  = zeros(1, N);

% Inicializamos el estado del observador en 0 (sin conocer el sesgo inicial)
x_hat(:, 1) = [0; 0; 0];

% Bucle de simulación muestra a muestra
for k = 1:N-1
    % La planta real sufre una deriva angular debido al sesgo: d(theta)/dt = omega + beta
    % Actualización física del sistema real (con bias real inyectado)
    x_real(3, k) = beta_real; 
    
    % Medición de ángulo con ruido
    y_med(k) = Cd * x_real(:, k) + sigma_ruido * randn();
    
    % Innovación del observador (Medición real - Estimación)
    e = y_med(k) - Cd * x_hat(:, k);
    
    % Actualización del Observador Aumentado: x_hat(k+1) = Ad*x_hat + Bd*u + L*e
    x_hat(:, k+1) = Ad * x_hat(:, k) + Bd * u(k) + L * e;
    
    % Evolución real del sistema (con la velocidad contaminada por beta)
    % A_real acopla x3 (beta) a la variación del ángulo x1
    A_real = Ad; 
    A_real(1, 3) = Ts; % Integración del sesgo sobre el ángulo real
    x_real(:, k+1) = A_real * x_real(:, k) + Bd * u(k);
end
y_med(N) = Cd * x_real(:, N) + sigma_ruido * randn();

%% 4. Visualización de Resultados
figure();

% Subplot 1: Ángulo real vs estimado
subplot(3,1,1);
plot(t, y_med, 'r.', 'DisplayName', 'Ángulo medido (y)'); hold on;
plot(t, x_hat(1,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ángulo estimado (\hat{\theta})');
grid on; ylabel('Ángulo [deg]'); title('Seguimiento del Observador Aumentado'); legend;

% Subplot 2: Velocidad Angular real vs estimada
subplot(3,1,2);
plot(t, x_real(2,:), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Velocidad real (\omega)'); hold on;
plot(t, x_hat(2,:), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Velocidad estimada (\hat{\omega})');
grid on; ylabel('Velocidad [deg/s]'); legend;

% Subplot 3: Estimación del Sesgo (Convergencia a beta_real)
subplot(3,1,3);
yline(beta_real, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Sesgo Real (\beta_{real})'); hold on;
plot(t, x_hat(3,:), 'g-', 'LineWidth', 2, 'DisplayName', 'Sesgo Estimado (\hat{x}_3)'); legend('Location', 'southeast');