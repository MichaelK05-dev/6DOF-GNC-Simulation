clear; clc; close all;

num_runs = 100;
modell_name = 'final_model.slx';
wind_north_std_dev = 3.0;
wind_east_std_dev  = 5.0;
thrust_std_dev = 0.02; % 2%
cd_std_dev = 0.10;     % 10%
cma_std_dev = 0.15;    % 15%

results_altitude = zeros(num_runs, 1);
results_final_pitch = zeros(num_runs, 1);
results_final_yaw = zeros(num_runs, 1);

for i = 1:num_runs
    V_wind = [wind_north_std_dev * randn(); wind_east_std_dev * randn(); 0];
    thrust_multiplier = 1 + thrust_std_dev * randn();
    cd_multiplier  = 1 + cd_std_dev * randn();
    cma_multiplier = 1 + cma_std_dev * randn();
    
    try
        simIn = Simulink.SimulationInput(modell_name);
        simIn = simIn.setVariable('V_wind', V_wind);
        simIn = simIn.setVariable('thrust_multiplier', thrust_multiplier);
        simIn = simIn.setVariable('cd_multiplier', cd_multiplier);
        simIn = simIn.setVariable('cma_multiplier', cma_multiplier);
        
        simOut = sim(simIn);
        logsout = simOut.logsout;
        height = logsout.get('height_signal');
        euler_angles = logsout.get('euler_angles_signal');
        
        results_altitude(i) = height.Values.Data(end);
        
        final_angles_rad = euler_angles.Values.Data(end, :);
        results_final_pitch(i) = rad2deg(final_angles_rad(2));
        results_final_yaw(i)   = rad2deg(final_angles_rad(3));
       
    catch ME
        fprintf(2, 'Error in Run %d: %s\n', i, ME.message);
        results_altitude(i) = NaN;
        results_final_pitch(i) = NaN;
        results_final_yaw(i) = NaN;
    end
end
fprintf('Ran successfully!\n');

nominal_altitude_meco = 83000;
nominal_pitch_meco = 31.0;
nominal_yaw_meco = 0.0;

figure('Name', 'Monte Carlo Analysis: State at MECO', 'NumberTitle', 'off', 'Position', [100, 100, 1400, 600]);

subplot(1, 2, 1);
histogram(results_altitude / 1000, 'NumBins', 15, 'FaceColor', '#0072BD');
title(sprintf('Altitude Distribution at MECO (%d Runs)', num_runs));
xlabel('Altitude at MECO (km)');
ylabel('Frequency');
grid on;
hold on;
actual_mean_altitude = mean(results_altitude / 1000);
actual_std_altitude = std(results_altitude / 1000); 
xline(actual_mean_altitude, 'r--', 'LineWidth', 2, 'Label', sprintf('Mean: %.1f km', actual_mean_altitude));
dim = [.2 .5 .3 .3]; % Position der Textbox [x y width height]
str = sprintf('Standard Deviation (1σ):\n%.2f km', actual_std_altitude);
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'BackgroundColor', 'white');
hold off;

subplot(1, 2, 2);
scatter(results_final_yaw, results_final_pitch, 36, 'b', 'filled', 'MarkerFaceAlpha', 0.5, 'DisplayName', 'Simulation Runs');
hold on;
plot(nominal_yaw_meco, nominal_pitch_meco, 'r+', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Nominal Target');

actual_mean_yaw = mean(results_final_yaw);
actual_std_yaw = std(results_final_yaw); 
actual_mean_pitch = mean(results_final_pitch);
actual_std_pitch = std(results_final_pitch);

stats_text = sprintf('1-Sigma Dispersion:\n  Yaw (σ_ψ): %.2f deg\n  Pitch (σ_θ): %.2f deg', ...
                     actual_std_yaw, actual_std_pitch);
text(0.05, 0.95, stats_text, 'Units', 'normalized', ...
     'VerticalAlignment', 'top', 'BackgroundColor', 'white', 'EdgeColor', 'black');

title(sprintf('Attitude Dispersion at MECO (%d Runs)', num_runs));
xlabel('Final Yaw Angle (degrees)');
ylabel('Final Pitch Angle (degrees)');
grid on;
axis equal;
legend('Location', 'best');
hold off;

fprintf('Plots generated.\n');
