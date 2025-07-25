%-------------------------------------------------------------------------%
% ----------------------- Proyecto Ball and Beam ------------------------ %
% --------------------------- UNLP - Control PD ------------------------- %
%-------------------------------------------------------------------------%

clc; clear; close all

%% Datos y función de transferencia

m = 0.111;
g = 9.81;            
R = 0.02;            
r = 0.014;           
J = 9.99e-6;

s = tf('s');

G = (m*g)/((J/R^2+m));
G = G * (0.0233/s^2);   % r en funcion de theta (no de alpha)

%% Control PD con filtro en derivada

% phi  = 0.5;   % [rad/s] cero del controlador
% beta = 10;     % [rad/s] polo del filtro derivativo
% 
% K = (s + phi)/(s + beta);

wt  = (1/20)*2*pi;   
eta = 0.7071;

% Parametrizacion alfin anda(?
p0 = -wt;
p1 = -wt*cos(eta) + wt*sin(eta)*1i;
p2 = -wt*cos(eta) - wt*sin(eta)*1i;

% Armo el polinomio denominador
Den = conv(poly(p1),poly(p2));
Den = Den(1)*s^2 + Den(2)*s + Den(3);
Den = Den * (s-p0);
De  = cell2mat(Den.Numerator);
a0  =  De(3)*s + De(4);

% Funcion Fq Final
Fq = (a0)/(Den);

% Control
K  = (Fq/(1-Fq)) * (1/G);
K  =  minreal(K, 0.1)

%% Lazos de control

LA  = K * G;        % Lazo abierto
S   = 1/(1 + LA);
T   = LA * S;

%% Respuestas

[y, t1] = step(0.15*T);              % Salida de la planta
[theta_rad] = step(0.2*K * S, t1);      % Acción de control (theta)

theta_deg = rad2deg(theta_rad) + 120;

%% Gráficos

figure(1)
subplot(211)
plot(t1, y * 100, 'LineWidth', 1.5)
xlabel('Tiempo [s]')
ylabel('Distancia [cm]')
title('Respuesta al escalón - Lazo Cerrado')
grid on

% Acción de control (theta)
subplot(212)
plot(t1, theta_deg, 'LineWidth', 1.5)
xlabel('Tiempo [s]')
ylabel('Ángulo de servo \theta [°]')
title('Acción de control (\theta)')
grid on

% PZMap lazo cerrado
figure(3)
pzmap(T)
title('Mapa de polos y ceros de T(s)')
grid on

% Sensibilidades
figure(4)
bode(T); hold on; bode(S)
title('Sensibilidades del lazo cerrado')
legend('T', 'S');
grid on

%% Control discreto

ts = 0.5;

opts = c2dOptions('Method', 'tustin', 'PrewarpFreq', 0.8*wt);
Kd   = c2d(K, ts, opts, wt)

%% Mapeo no lineal alpha -> theta   

alp = linspace(-40, 40, 60);
aprox = 0.0233 * alp;
plot(alp, alpha2theta(alp), 'b', alp, aprox, 'r'); 
grid on

function theta_rad = alpha2theta(alpha_deg)

    alpha_rad = deg2rad(alpha_deg);

    % Geometría del sistema (en metros)
    a = 0.03;    % [m]
    b = 0.08;    % [m]
    c = 0.04;    % [m]
    h = 0.08;    % [m]
    y = c - a;   % [m]

    % Calculo de variables intermedias
    r = sqrt( (h - c*sin(alpha_rad)).^2 + (c*cos(alpha_rad) - y).^2 );
    phi = acos( (a^2 + r.^2 - b^2) ./ (2*a*r) );
    gamma = asin( (c*cos(alpha_rad) - y) ./ r );

    % Ángulo absoluto del motor en radianes
    theta_rad = -(pi/2) + gamma + phi;

end
