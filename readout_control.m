clc
close all
clear

% Legge il nuovo CSV
T = readtable("20260123_103213.csv");

% Filtra solo ALTITUDE HOLD
T_alt = T(strcmp(T.mode,"ALTITUDE_HOLD"), :);

% --- Ricalcolo v_raw da z_meas (se vuoi confrontare) ---
dt2 = diff(T_alt.millis) * 0.001;
v_raw2 = diff(T_alt.z_meas) ./ dt2;

T = 0.7958;
alpha_v = 0.1 / (T + 0.1);  
N= length(dt2);
v_hat  = zeros(1,N);

for k = 2:N-1
    v_hat(k) = alpha_v * T_alt.v_raw(k) + (1 - alpha_v) * T_alt.v_hat(k-1);
end

Fs=1/0.1;
L = height(T_alt);
Y = fft(T_alt.v_raw);

y = lowpass(T_alt.v_raw,0.2,Fs);

figure()
plot(Fs/L*(0:L-1),abs(Y),"LineWidth",3)
title("Complex Magnitude of fft Spectrum")
xlabel("f (Hz)")
ylabel("|fft(X)|")

M = 5;
z = T_alt.v_raw;
N = length(z);

z_f = zeros(size(z));

for k = 1:N
    i_start = max(1, k-M+1);   % evita indici negativi
    z_f(k) = mean(z(i_start:k));
end

figure
plot(T_alt.millis, z, 'DisplayName','z raw'); hold on;
plot(T_alt.millis, z_f, 'DisplayName','z filtered');
legend; grid on;
title('Media Mobile');




%% ============================
%          PLOT 1
% ==============================
figure
plot(T_alt.millis, T_alt.z_meas, '.', 'DisplayName', 'z\_meas');
hold on
plot(T_alt.millis, T_alt.z_ref, 'DisplayName', 'z\_ref');

xlabel('Time (ms)');
ylabel('Altitude (m)');
title('Altitude Hold: z meas vs z ref');
ylim([0 2]);
grid on
legend;

%% ============================
%          PLOT 2
% ==============================
figure
plot(T_alt.millis, T_alt.v_hat, 'DisplayName','v\_hat');
hold on
plot(T_alt.millis, T_alt.v_ref, 'DisplayName','v\_ref');
plot(T_alt.millis, T_alt.v_raw, 'DisplayName','v\_raw (log)');
plot(T_alt.millis(2:end), v_raw2, 'DisplayName','v\_raw (recalc)');

xlabel('Time (ms)');
ylabel('Vertical velocity (m/s)');
title('Velocity Tracking');
legend();
grid on;

figure
plot(T_alt.millis, T_alt.v_hat, 'DisplayName','v\_hat');
hold on
plot(T_alt.millis(2:end), v_hat, 'DisplayName','v\_hat_calc');
plot(T_alt.millis, y, 'DisplayName','v\_hat_calc');
xlabel('Time (ms)');
ylabel('Vertical velocity (m/s)');
title('Velocity Tracking');
legend();
grid on;