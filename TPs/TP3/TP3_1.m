clc;clear;
%% FUNCIONES DE TRANSFERENCIA
s = tf('s');
G_sb = 56.28/(s^2 + 16.61*s + 158.9);
G_bc = 13.3/(s^2 + 3.5450*s);
G_total = G_sb*G_bc;

%% SUBSISTEMA SERVO-BARRA

A_sb = [0       1;
       -158.9  -16.61];

B_sb = [0;
        56.28];

C_sb = [1  0];

D_sb = 0;


%% SUBSISTEMA BARRA-CARRITO

A_bc = [0  1;
        0 -3.5450];

B_bc = [0;
        13.3];

C_bc = [1  0];

D_bc = 0;


%% MODELO COMPLETO [p, dp, theta, dtheta] (INTERCONEXION EN SERIE)

A = [A_bc          B_bc*C_sb;
     zeros(2,2)    A_sb];

B = [zeros(2,1);
     B_sb];

C = [1  0  0  0;
     0  0  1  0];

D = [0;
     0];

%% MODELO EN VARIABLES DE ESTADO
sys_sb = ss(A_sb,B_sb,C_sb,D_sb);
sys_bc = ss(A_bc,B_bc,C_bc,D_bc);
sys = ss(A,B,C,D);

%% MATRIZ DE OBSERVABILIDAD

O = obsv(A,C);

rango_O = rank(O);

disp('Matriz de observabilidad:');
disp(' ');
pretty(vpa(sym(O),5));

disp('Rango de la matriz de observabilidad:');
disp(rango_O);

%% OBSERVABILIDAD MIDIENDO SOLAMENTE p

C_p = [1  0  0  0];

O_p = obsv(A,C_p);

rango_p = rank(O_p);

disp(['Rango midiendo solamente la posicion: ', num2str(rango_p) ]);

%% OBSERVABILIDAD MIDIENDO SOLAMENTE theta

C_theta = [0  0  1  0];

O_theta = obsv(A,C_theta);

rango_theta = rank(O_theta);

disp(['Rango midiendo solamente el angulo: ', num2str(rango_theta)]);



%% DISCRETIZACION POR FORWARD 

Ts = 0.02;

Ad = eye(4) + Ts*A;
Bd = Ts*B;

Cd = C;
Dd = D;

sys_forward = ss(Ad,Bd,Cd,Dd,Ts);


%% DISCRETIZACION POR ZOH

sys_c = ss(A,B,C,D);

sys_zoh = c2d(sys_c,Ts,'zoh');

Ad_exacto = sys_zoh.A;
Bd_exacto = sys_zoh.B;
Cd_exacto = sys_zoh.C;
Dd_exacto = sys_zoh.D;

%% SIMULACION

t = 0:Ts:5;

% u = ones(length(t),1);  % Entrada escalon unitario
u = zeros(length(t),1);   % Entrada impulso
u(1) = 1/Ts;

% Sistema continuo
[y_c,t_c] = lsim(sys_c,u,t);

% Forward Euler
[y_forward,t_forward] = lsim(sys_forward,u,t);

% ZOH
[y_zoh,t_zoh] = lsim(sys_zoh,u,t);


%% COMPARACION DE LA POSICION

figure;

plot(t_c,y_c(:,1),'b','LineWidth',1.5);
hold on;
stairs(t_forward,y_forward(:,1),'r','LineWidth',1.5);
stairs(t_zoh,y_zoh(:,1),'g--','LineWidth',1.5);

grid on;
xlabel('Tiempo [s]');
ylabel('Posicion p');
title('Comparacion de la posicion del carrito');

legend('Continuo','Forward Euler','ZOH');


%% COMPARACION DEL ANGULO

figure;

plot(t_c,y_c(:,2),'b','LineWidth',1.5);
hold on;
stairs(t_forward,y_forward(:,2),'r','LineWidth',1.5);
stairs(t_zoh,y_zoh(:,2),'g--','LineWidth',1.5);

grid on;
xlabel('Tiempo [s]');
ylabel('Angulo \theta');
title('Comparacion del angulo de la barra');

legend('Continuo','Forward Euler','ZOH');

%% DISEÑO DEL OBSERVADOR

polos_CON = eig(A);
disp('Polos del sistema continuo:');
disp(" ");
polos_CON(1)=-1;
disp(polos_CON);


polos_RAPIDOS = 2*polos_CON;
polos_discretos = exp(polos_RAPIDOS*Ts);

disp('Polos discretos:');
disp(" ");
disp(polos_discretos);

%L = place(Ad_exacto',Cd_exacto',polos_discretos)';
L = place(Ad',Cd',polos_discretos)';
disp('Matriz de ganancias L:');
disp(" ");
pretty(vpa(sym(L),5));

%VERIFICO
polos_observador = eig(Ad-L*Cd);
disp('Polos obtenidos del observador:');
disp(" ");
disp(polos_observador);

%% VALOSRES DE K

% Polos deseados continuo
polos_continuos = [-2, -3, -8+8j, -8-8j];

% Conversion a discreto
polos_deseados = exp(polos_continuos*Ts);

%K = -acker(Ad_exacto, Bd_exacto, polos_deseados);
%Acl = Ad_exacto + Bd_exacto*K;
K = -acker(Ad, Bd, polos_deseados);
Acl = Ad + Bd*K;


eig(Acl)
disp('Valores de K:');
pretty(vpa(sym(K),5));

%% ========================================================================
%% AGREGADO AL FINAL DE TP3_1.m: CONTROL POR REALIMENTACIÓN DE ESTADOS + ACCIÓN INTEGRAL
%% ========================================================================

% 1. Sistema discreto aumentado (4 estados de la planta + 1 estado integrador q_k)
% Usa C_p = [1 0 0 0], Ad_exacto, Bd_exacto y Ts ya definidos arriba en TP3_1.m
A_aug = [Ad,   zeros(4,1);
        -C_p * Ts,    1];

B_aug = [Bd;
         0];

% Verificación de controlabilidad del sistema aumentado
disp('Rango de controlabilidad del sistema aumentado:');
disp(rank(ctrb(A_aug, B_aug)));

% 2. Polos deseados en continuo para cumplir ts < 2.5 s y Mp < 15%
% Par dominante: zeta = 0.65 (Mp ~ 6.8%), wn = 3.2 rad/s (ts ~ 1.92 s)
zeta = 0.65;
wn   = 3.2;
sigma = zeta * wn;
wd    = wn * sqrt(1 - zeta^2);

polos_continuos_int = [
    -sigma + 1j*wd;   % Polo dominante 1
    -sigma - 1j*wd;   % Polo dominante 2
    -5.5;             % Polo integral
    -8 + 8j;          % Polos rápidos de la barra (misma frecuencia que tu K anterior)
    -8 - 8j
];

polos_deseados_int = exp(polos_continuos_int * Ts);

% 3. Cálculo de K_aug = [K_base, H] con tu convención u = K*x + H*q
K_aug = -acker(A_aug, B_aug, polos_deseados_int);

K_int = K_aug(1:4);
H     = K_aug(5);

% Compensación porque en anidamiento_mas_accion_integral_TP3.ino se actualiza q_k
% ANTES de calcular u_k: u_k = K*x_k + H*(q_k + Ts*(ref - p_k))
K_int(1) = K_int(1) + H * Ts;

% Verificación de los polos reales de lazo cerrado tal como corre en el .ino
Acl_ino = [Ad + Bd*(K_int - H*Ts*C_p),  Bd*H;
          -C_p*Ts,                                   1];

disp('Polos reales a lazo cerrado en z (con orden del .ino):');
disp(eig(Acl_ino));

disp('Valores para copiar en anidamiento_mas_accion_integral_TP3.ino:');
fprintf('Matrix<1, 4> mat_K = { %.6f , %.6f , %.6f , %.6f };\n', K_int(1), K_int(2), K_int(3), K_int(4));
fprintf('Matrix<1, 1> H = { %.6f };\n\n', H);

%% 4. SIMULACIÓN CON OBSERVADOR Y SATURACIÓN DEL SERVO (ESCALÓN 0 -> 7 cm)
N_sim = length(t);
p_ref = 7 * ones(1, N_sim);

x_sim     = zeros(4, N_sim);
x_hat_sim = zeros(4, N_sim);
q_sim     = 0;
u_sim     = zeros(1, N_sim);

for k = 1:N_sim-1
    y_k     = Cd_exacto * x_sim(:, k);
    y_hat_k = Cd_exacto * x_hat_sim(:, k);
    
    % Exactamente las mismas líneas que en tu loop() de Arduino
    q_sim = q_sim + Ts * (p_ref(k) - x_hat_sim(1, k));
    u_raw = K_int * x_hat_sim(:, k) + H * q_sim;
    
    % Saturación física: constrain(93.0 - u_k, 50.0, 160.0) -> u_k en [-67, 43]
    u_sim(k) = max(min(u_raw, 43), -67);
    
    % Evolución de planta y observador
    x_sim(:, k+1)     = Ad_exacto * x_sim(:, k) + Bd_exacto * u_sim(k);
    x_hat_sim(:, k+1) = Ad_exacto * x_hat_sim(:, k) + Bd_exacto * u_sim(k) + L * (y_k - y_hat_k);
end
u_sim(N_sim) = u_sim(N_sim-1);

info_int = stepinfo(x_sim(1,:), t, 7);
fprintf('--- DESEMPEÑO SIMULADO ---\n');
fprintf('Tiempo de establecimiento (2%%): %.3f s\n', info_int.SettlingTime);
fprintf('Sobrepico porcentual:           %.2f %%\n', info_int.Overshoot);
fprintf('Pico de esfuerzo de control:    %.2f deg\n', max(abs(u_sim)));

figure;
subplot(2,1,1);
plot(t, p_ref, 'k--', 'LineWidth', 1.2); hold on;
plot(t, x_sim(1,:), 'b', 'LineWidth', 1.5);
plot(t, x_hat_sim(1,:), 'r:', 'LineWidth', 1.5);
grid on;
ylabel('Posicion p [cm]');
title('Control integral con observador (Simulacion TP3)');
legend('Referencia', 'p real', 'p estimada', 'Location', 'southeast');

subplot(2,1,2);
stairs(t, u_sim, 'm', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Accion de control u [deg]');