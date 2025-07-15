%-------------------------------------------------------------------------%
%----------------------- Proyecto ball and beam --------------------------%
%-------------------------------- UNLP -----------------------------------%

clc; clear
%% Datos y funcion de transferencia

g = 9.81;
s = tf([1 0], 1);
R = 0.02;
r = 0.014;

G = g / (1 + (2/5)*(R/r)^2);
G = G * (1/s^2);

%% Relación de ángulo

a = 0.03; % distancia de centro a biela
b = 0.08; % largo de biela
c = 0.04; % dist entre pivot y biela
h = 0.08; % distancia vertical centro de giro y pivot
y = c -a; % distancia horizontal entre ejes de giros

x = linspace(-40, 40, 80) * pi/180;
% x = 0;

r   = sqrt( (h - c*sin(x)).^2 + (c*cos(x) - y).^2 );
phi = acos( (a^2 + r.^2 - b^2)./ (2*a*r) );
gam = asin( (c*cos(x) - y )./r);

theta = -(pi/2) + gam + phi;

plot(x*180/pi, theta*180/pi); grid on