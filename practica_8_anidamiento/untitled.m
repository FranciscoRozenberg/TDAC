%% Practica 8 - Anidamiento
% Matrices de estados 

Ad = [ 0.9718 0.01684 ;                 
      -2.661,  0.6921 ];

Bd = [ 0.01005 ;
       0.9477  ];

Cd = [1.0, 0];

Dd = [0];

L = [ 0.4188 ;
      1.298  ];
%%
K = acker(Ad, -Bd, [.75;.75])

%%
F = (C *(eye(2) - (Ad + Bd *K))^-1 *Bd )^-1

%% Simulación recursiva del Observador de Estados Discreto
% Se utiliza un bucle explícito para iterar las ecuaciones de estado

% Definición del vector de tiempo discreto basado en el escalón
t_sim = 0:Ts:5;                % Simulación de 0 a 5 segundos
N = length(t_sim);
r = ones(1, N);                % Entrada escalón unitario

% Generación del ruido de medición
sigma_v = 0.1;
v = sigma_v * randn(1, N);     % Ruido blanco gaussiano

% Inicialización de vectores de estado y salidas
x_real = zeros(2, N);          % Estado real: [ángulo; velocidad]
x_hat  = zeros(2, N);          % Estado estimado por el observador
y_real = zeros(1, N);          % Salida real (sin ruido)
y_r    = zeros(1, N);          % Salida medida (con ruido)
y_hat  = zeros(1, N);          % Salida estimada

% Bucle explícito de simulación (x_n+1 = A*x_n + ...)
for n = 1:N-1
    % --- Salida y Medición en el instante n ---
    y_real(n) = Cd * x_real(:, n);
    y_r(n)    = y_real(n) + v(n);
    y_hat(n)   = Cd * x_hat(:, n);

    % --- Ecuación de Estado de la Planta Real ---
    % x_real(n+1) = Ad * x_real(n) + Bd * u(n)
    x_real(:, n+1) = (Ad + Bd *K ) *x_real(:, n) + Bd * F *r(n);

    % --- Ecuación de Estado del Observador Discreto ---
    % x_hat(n+1) = Ad * x_hat(n) + Bd * u(n) + L * ( y_r(n) - Cd * x_hat(n) )
    x_hat(:, n+1) = (Ad + Bd *K ) *x_hat(:, n) + Bd *F *r(n) + L * (y_r(n) - y_hat(n));
end

% Cálculo del último punto de salida
y_real(N) = Cd * x_real(:, N);
y_r(N)    = y_real(N) + v(N);
y_hat(N)  = Cd * x_hat(:, N);

Acl = Ad + Bd*K
closedLoopPoles = eig(Acl)

%%

figure;
plot(t_sim, y_real, 'b', t_sim, y_r, 'x', t_sim, y_hat, 'r--');
grid on;
xlabel('Tiempo (s)');
ylabel('Salida');
legend('Salida real', 'Medición', 'Salida estimada');