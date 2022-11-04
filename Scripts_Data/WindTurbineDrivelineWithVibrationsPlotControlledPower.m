% Code to plot contoller outputs and wind turbine response versus wind speed for WindturbineDrivleineWithVibrations
%% Plot Description:
%
% This script plots the wind speed, generated power, turbine rotational
% velocity, controlled collective blade pitch, and controlled generator.

% Copyright 2022 The MathWorks, Inc.

% Generate simulation results if they don't exist
if ~exist('WindTurbineOut', 'var')
    WindTurbineOut= sim('WindTurbineDrivelineWithVibrations');
end

% Reuse figure if it exists, else create new figure
if ~exist('h1_WindTurbineDrivelineWithVibrations', 'var') || ...
        ~isgraphics(h1_WindTurbineDrivelineWithVibrations, 'figure')
    h1_WindTurbineDrivelineWithVibrations = figure('Name', 'WindTurbineDrivelineWithVibrations');
end
figure(h1_WindTurbineDrivelineWithVibrations);
clf(h1_WindTurbineDrivelineWithVibrations);

plotControlledResponse(WindTurbineOut.simlog, h1_WindTurbineDrivelineWithVibrations);

% Create plot from simulation results
function plotControlledResponse(simlog, hFigure)

% Get simulation results:
t = simlog.Blade_Torque.vWind.series.time;
vWind= simlog.Blade_Torque.vWind.series.values('m/s');
generatedPower= simlog.Controlled_Generator.Rotational_Power_Sensor.instantaneousPower.series.values('MW');
turbineVelocity= simlog.Blade_Torque.w.series.values('rpm');
bladePitch= simlog.Blade_Torque.pitch.series.values('deg');
generatorTorque= simlog.Controlled_Generator.Ideal_Torque_Source.t.series.values('kN*m');


% Plot results:
subplot(5,1,1);
plot(t,vWind,'LineWidth',1);
grid on
ylabel('Wind speed');
xlabel('Time (s)');
title('Wind Speed');

subplot(5,1,2);
plot(t,generatedPower,'LineWidth',1);
grid on
ylabel('Power (MW)');
xlabel('Time (s)');
title('Generated Power');

subplot(5,1,3);
plot(t,turbineVelocity,'LineWidth',1);
grid on
ylabel('Velocity (RPM)');
xlabel('Time (s)');
title('Turbine Rotational Velocity');

subplot(5,1,4);
plot(t,bladePitch,'LineWidth',1);
grid on
ylabel('Pitch (deg)');
xlabel('Time (s)');
title('Collective Blade Pitch');

subplot(5,1,5);
plot(t,generatorTorque./10^3,'LineWidth',1);
grid on
ylabel('Torque (MN*m)');
xlabel('Time (s)');
title('Generator Torque');

% Increase figure height, while holding figure top position constant
minFigureHeight= 630; % [pixels]
amtToIncreaseHeight= max(minFigureHeight, hFigure.Position(4)) - hFigure.Position(4);
hFigure.Position= [hFigure.Position(1), hFigure.Position(2)-amtToIncreaseHeight,...
                   hFigure.Position(3), hFigure.Position(4)+amtToIncreaseHeight]; % [left, bottom, width, height]

end