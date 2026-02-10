clc; close all; clear;

z_ref = 1; % meters

modelName = 'vert_control_sim_2024b';

% Open the Simulink model visually
open_system(modelName);
set_param(modelName, 'StopTime', '150');  

% Run the simulation
simOut = sim(modelName);

% Read output signals
t       = simOut.tout;
z       = simOut.z;
vz      = simOut.v_z;
vz_hat  = simOut.vz_hat;
vz_ref  = simOut.v_z_ref;
u_pid   = simOut.u_PID_sat;

% --- SUBPLOTS ---
figure;

% ---- Altitude ----
subplot(3,1,1);
plot(t, z, 'LineWidth', 1.5, 'DisplayName', 'z'); hold on;
yline(z_ref, 'LineWidth', 1.5, 'Color', 'k', 'DisplayName', 'z_{ref}');
grid on;
xlabel('Time [s]','FontSize',12);
ylabel('Altitude z [m]','FontSize',12);
% title('Altitude Response');
legend('Location','best','FontSize',12);

% ---- Vertical velocity ----
subplot(3,1,2);
plot(t, vz, 'LineWidth', 1.5, 'DisplayName', 'v_{z,real}'); hold on;
plot(t, vz_ref, 'LineWidth', 1.5, 'DisplayName', 'v_{z,ref}');
plot(t, vz_hat, 'LineWidth', 1.5, 'DisplayName', 'v_{z,hat}');
grid on;
xlabel('Time [s]','FontSize',12);
ylabel('Velocity v_z [m/s]','FontSize',12);
% title('Velocity Response');
legend('Location','best','FontSize',12);

% ---- Control input ----
subplot(3,1,3);
plot(t, u_pid, 'LineWidth', 1.5, 'DisplayName', 'u_{PID}');
grid on;
xlabel('Time [s]','FontSize',12);
ylabel('u_{PID}','FontSize',12);
% % title('Control Input');
legend('Location','best','FontSize',12);

disp('Simulation completed.');

disp('Simulation completed.');
