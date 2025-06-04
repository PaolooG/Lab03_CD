
%% LABORATORIO 3 - MODULACION PASABANDA DE SEÑALES BINARIAS
% Autor: Auto-generado
% Experimentos con OOK (ASK) y FSK en MATLAB

clc; clear; close all;

%% Parámetros Generales
Rb = 1000;             % Tasa de bits (bps)
Tb = 1 / Rb;           % Duración de bit
fs = 100e3;            % Frecuencia de muestreo
N = 100;               % Número de bits
t_bit = 0:1/fs:Tb - 1/fs; % Tiempo para un bit

%% Generación de bits y señal digital
bits = randi([0 1], 1, N);     % Bits aleatorios
pulse = ones(1, length(t_bit)); % Pulso rectangular
x = kron(bits, pulse);         % Tren de pulsos NRZ unipolar
t = (0:length(x)-1)/fs;

%% Modulación OOK (ASK)
fc = 10e3;                      % Frecuencia portadora
s_ook = x .* cos(2*pi*fc*t);    % Modulación OOK

%% Envolvente Compleja y Espectro de OOK
g_ook = x;                     % Envolvente real para OOK
G_f = fftshift(abs(fft(g_ook, 2^nextpow2(length(g_ook)))));
f = linspace(-fs/2, fs/2, length(G_f));

figure; plot(f, G_f);
xlabel('Frecuencia (Hz)'); ylabel('|G(f)|');
title('Espectro de la envolvente OOK');

%% Modulación FSK
f1 = 9e3; f2 = 11e3;            % Frecuencias FSK para 0 y 1
s_fsk = zeros(1, length(x));

for i = 1:N
    f_bit = bits(i)*f2 + (1-bits(i))*f1;
    idx = (i-1)*length(pulse)+1:i*length(pulse);
    s_fsk(idx) = pulse .* cos(2*pi*f_bit*t(idx));
end

%% Envolvente Compleja y Espectro de FSK
g_fsk = hilbert(s_fsk);       % Envolvente compleja g(t)
G_fsk = fftshift(abs(fft(g_fsk, 2^nextpow2(length(g_fsk)))));
f = linspace(-fs/2, fs/2, length(G_fsk));

figure; plot(f, G_fsk);
xlabel('Frecuencia (Hz)'); ylabel('|G(f)|');
title('Espectro de la envolvente FSK');

%% Cálculo teórico de ancho de banda
BW_ASK = Rb;
delta_f = abs(f2 - f1);
BW_FSK = 2 * delta_f + Rb;

fprintf('Ancho de banda teórico OOK: %.2f Hz\n', BW_ASK);
fprintf('Ancho de banda teórico FSK: %.2f Hz\n', BW_FSK);
