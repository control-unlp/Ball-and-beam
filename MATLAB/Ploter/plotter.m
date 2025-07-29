clc; clear

%% Acomodo los datos

[t, y] = leer_log('log_20250729_113757.csv');

dist  = y(:, 1);
err   = y(:, 1);
angle = y(:, 1);

%% Ploteos

figure(1)
plot(t, dist)
title('Medición del sensor laser')
xlabel('Tiempo [s]');
ylabel('Distancia horizontal [cm]');
grid on
ylim([-5, 80]);


% figure(1)
% plot(t, dist)
% title('Medición del sensor laser')
% xlabel('Tiempo [s]');
% ylabel('Distancia horizontal [cm]');
% grid on
% ylim([-5, 80]);
