% Code to plot simulation results from the WindTurbineDrivelineWithVibrations demo
%% Plot description:
%
% This code varies the number of degrees of freedom for each vibration
% model and plots the computed response and execution time.
% The plot can be configured to show force, moment, translational velocity,
% or rotational velocity at the bearings.

% Copyright 2022 The MathWorks, Inc.

% Reuse figures if they exist, else create new figures
if ~exist('h5_WindTurbineDrivelineWithVibrations', 'var') || ...
        ~isgraphics(h5_WindTurbineDrivelineWithVibrations, 'figure')
    h5_WindTurbineDrivelineWithVibrations = figure('Name', 'h5_WindTurbineDrivelineWithVibrations');
end

if ~exist('h6_WindTurbineDrivelineWithVibrations', 'var') || ...
        ~isgraphics(h6_WindTurbineDrivelineWithVibrations, 'figure')
    h6_WindTurbineDrivelineWithVibrations = figure('Name', 'h5_WindTurbineDrivelineWithVibrations');
end


getPerformance(h5_WindTurbineDrivelineWithVibrations, h6_WindTurbineDrivelineWithVibrations,...
    'WindTurbineDrivelineWithVibrations', 'WindTurbineDrivelineWithVibrations/Shaft')

%%
function getPerformance(hFigure1, hFigure2, model, block)

%% Set-up
hws= get_param(model, 'modelworkspace');
fs= getVariable(hws, 'fs');
fs.ne_Lumped= 1;
scenario= getVariable(hws, 'scenario');

% Specify node locations (and therefore the minimum number of nodes) for the lumped mass model
z_shaft= {...
    linspace(0, fs.length_shaft, 6);
    linspace(0, fs.length_shaft, 11)};

% Specify the number of modes for the static and speed-dependent eigenmodes methods
num_modes_vector= [12 16 20 24];

% Specify number of inner modes for the Craig-Bampton method
num_Craig_Bampton_inner_modes_vector= [0 4 8];

% Specify the number of simulation iterations used to get the average execution time
num_iterations= 1;

%% No transerve flexibility
disp('Getting rigid results')

set_param(block, 'vibration_model', 'Rigid')

for j=1:num_iterations
    disp(['Simulation ' num2str( j ) ' of ' num2str(num_iterations)]);

    % Simulate
    out= sim(model);

    % Get results
    execution_times(j)= out.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime; %#ok<AGROW> 
    disp(['Execution time: ' num2str(j) ': ' num2str(execution_times(j)) ' s']);
end

%% Lumped mass model
disp('Getting Lumped mass results')

set_param(block, 'vibration_model', 'Lumped mass')

num_Lumped_variations= size(z_shaft,1);

% Update number of elements
for i= 1:num_Lumped_variations
       execution_times= zeros(1, num_iterations);

    z_shaft_i= z_shaft{i};
    set_param(block, 'z_shaft', ['[' num2str(z_shaft_i) ']' ]);

     disp(['Simulation ' num2str(i) ' of ' num2str(num_Lumped_variations) '. ' ...
        num2str(length(z_shaft_i)-1) ' flexible elements']);

    for j= 1
        % Simulate
         out= sim(model);

        % Get results
        execution_times(j)= out.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;
        disp(['Execution time: ' num2str(execution_times(j)) ' s']);
    end
    Lumped.t(i)= mean(execution_times);

    Vsupports= out.simlog.Shaft.Lumped_mass.Flexible_Shaft_Dynamics.Vsupports.series.values('m/s');
    Msupports= out.simlog.Shaft.Lumped_mass.Flexible_Shaft_Dynamics.Msupports.series.values('N*m');
    Fsupports= out.simlog.Shaft.Lumped_mass.Flexible_Shaft_Dynamics.Fsupports.series.values('N');

    Lumped.Vsupports(1:2*fs.num_supports,i)= max(Vsupports);
    Lumped.Msupports(1:2*fs.num_supports,i)= max(Msupports);
    Lumped.Fsupports(1:2*fs.num_supports,i)= max(Fsupports);
end

%% Static eigenmodes model
disp('Getting Static eigenmodes results')

set_param(block, 'vibration_model', 'Static eigenmodes')

% Update number of modes
num_mode_variations= length( num_modes_vector );
for i= 1:num_mode_variations
    execution_times= zeros(1, num_iterations);

    % Set model parameters
    num_modes= num_modes_vector(i);
    set_param(block, 'num_modes_eig', num2str(num_modes));  

    for j= 1:num_iterations
        disp(['Simulation ' num2str((i-1)*num_iterations+j ) ' of ' num2str(num_mode_variations*num_iterations) '. ' ...
        num2str(num_modes) ' modes']);
    
        % Simulate
        out= sim(model);
    
        % Get results
        execution_times(j)= out.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;
        disp(['Execution time: ' num2str(execution_times(j)) ' s']);
    end
    Static.t(i)= mean(execution_times);

    Vsupports= out.simlog.Shaft.Static_eigenmodes.Flexible_Shaft_Dynamics.Vsupports.series.values('m/s');
    Msupports= out.simlog.Shaft.Static_eigenmodes.Flexible_Shaft_Dynamics.Msupports.series.values('N*m');
    Fsupports= out.simlog.Shaft.Static_eigenmodes.Flexible_Shaft_Dynamics.Fsupports.series.values('N');

    Static.Vsupports(1:2*fs.num_supports,i)= max(Vsupports);
    Static.Msupports(1:2*fs.num_supports,i)= max(Msupports);
    Static.Fsupports(1:2*fs.num_supports,i)= max(Fsupports);
end

%% Speed-dependent eigenmodes model
disp('Getting Speed-dependent eigenmodes results')

set_param(block, 'vibration_model', 'Speed-dependent eigenmodes')

% Update number of modes
num_mode_variations= length( num_modes_vector );
for i= 1:num_mode_variations
    execution_times= zeros(1, num_iterations);

    % Set model parameters
    num_modes= num_modes_vector(i);
    set_param(block, 'num_modes_eig', num2str(num_modes));

    for j= 1:num_iterations
        disp(['Simulation ' num2str((i-1)*num_iterations+j ) ' of ' num2str(num_mode_variations*num_iterations) '. ' ...
        num2str(num_modes) ' modes']);

        % Simulate
        out= sim(model);

        % Get results
        execution_times(j)= out.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;
        disp(['Execution time: ' num2str(execution_times(j)) ' s']);
    end
    SpeedDep.t(i)= mean(execution_times);

    Vsupports= out.simlog.Shaft.Speed_dependent_eigenmodes.Flexible_Shaft_Dynamics.Vsupports.series.values('m/s');
    Msupports= out.simlog.Shaft.Speed_dependent_eigenmodes.Flexible_Shaft_Dynamics.Msupports.series.values('N*m');
    Fsupports= out.simlog.Shaft.Speed_dependent_eigenmodes.Flexible_Shaft_Dynamics.Fsupports.series.values('N');

    SpeedDep.Vsupports(1:2*fs.num_supports,i)= max(Vsupports);
    SpeedDep.Msupports(1:2*fs.num_supports,i)= max(Msupports);
    SpeedDep.Fsupports(1:2*fs.num_supports,i)= max(Fsupports);
end

%% Craig-Bampton model
disp('Getting Craig-Bampton results')

set_param(block, 'vibration_model', 'Craig-Bampton')

% Update number of inner modes
num_mode_variations= length( num_Craig_Bampton_inner_modes_vector );
for i= 1:num_mode_variations
    execution_times= zeros(1, num_iterations);

    % Set model parameters
    num_inner_modes= num_Craig_Bampton_inner_modes_vector(i);
    set_param(block, 'num_modes_CB', num2str(num_inner_modes));
    

    for j= 1:num_iterations
        disp(['Simulation ' num2str((i-1)*num_iterations+j ) ' of ' num2str(num_mode_variations*num_iterations) '. ' ...
        '20 static modes. ' num2str(num_inner_modes) ' inner modes']);

        % Simulate
        out= sim(model);
    
        % Get results
        execution_times(j)= out.SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;
        disp(['Execution time: ' num2str(execution_times(j)) ' s']);
    end
    CB.t(i)= mean(execution_times);

    Vsupports= out.simlog.Shaft.Craig_Bampton.Flexible_Shaft_Dynamics.Vsupports.series.values('m/s');
    Msupports= out.simlog.Shaft.Craig_Bampton.Flexible_Shaft_Dynamics.Msupports.series.values('N*m');
    Fsupports= out.simlog.Shaft.Craig_Bampton.Flexible_Shaft_Dynamics.Fsupports.series.values('N');

    CB.Vsupports(1:2*fs.num_supports,i)= max(Vsupports);
    CB.Msupports(1:2*fs.num_supports,i)= max(Msupports);
    CB.Fsupports(1:2*fs.num_supports,i)= max(Fsupports);
end

%% Plot results
figure(hFigure1); figure(hFigure2);
clf(hFigure1); clf(hFigure2)

% Plot Force in X and Y directions for first bearing
results_to_plot=        {'Fsupports', 'Fsupports'};
signal_index_to_plot=   [1,            2];   
titles=                 {'First Bearing- Maximum Horizontal Force Amplitude', 'First Bearing- Maximum Vertical Force Amplitude'};
ylabels=                {'% Error',         '% Error'};

h_tiledLayout= zeros(2,1);
showXlabel= [1 1]; % Flag for showing X labels in each tile
showLegend= [1 0]; % Flag for showing legend in each tile

% Accuracy threshold for defining the most computationally efficient accurate result 
best_error_threshold= 0.02;

for i=1:length(results_to_plot)

   %% Figure 1: Plot % Error versus Run/Sim Time Ratio
   figure(hFigure1)

   ref_result= Lumped.(results_to_plot{i})(signal_index_to_plot(i),end);
   sim_time= scenario.tEnd;

    h_tiledLayout(i)= nexttile;
    title(titles{i})
    hold on
    hLines= zeros(1,length(results_to_plot)*length(signal_index_to_plot));
    
    % Plot thick axis at error of 0
    plot([0 1.1*Lumped.t(end)/sim_time], [0 0], 'Linewidth', 0.5, 'Color', 'Black')
    xlim([0 1.1*Lumped.t(end)/sim_time])
    
    % Plot lumped mass results
    Lumped_Error= (Lumped.(results_to_plot{i})(signal_index_to_plot(i),:)-ref_result)./ref_result*100;
    Lumped_time_ratio= Lumped.t./sim_time;
    hLines(1)= plot(Lumped_time_ratio, Lumped_Error, '+-', 'Color', [0 0.45 0.74], 'MarkerSize', 5, 'DisplayName','Lumped Mass');
    % Determine the best result
    [Lumped_best_time_ratio, Lumped_best_error]= getBestResult(Lumped_Error, Lumped_time_ratio, best_error_threshold);
    hLines(5)= plot(Lumped_best_time_ratio, Lumped_best_error, 'ko', 'DisplayName','Best Result for Each Method');
    
    % Plot static eigenmode results
    Static_Error= (Static.(results_to_plot{i})(signal_index_to_plot(i),:)-ref_result)./ref_result*100;
    Static_time_ratio= Static.t./sim_time;
    hLines(2)= plot(Static_time_ratio, Static_Error, 'x--', 'Color', [0.93 0.69 0.13],  'DisplayName', 'Static Eigenmodes');
    % Determine the best result
    [Static_best_time_ratio, Static_best_error]= getBestResult(Static_Error, Static_time_ratio, best_error_threshold);
    plot(Static_best_time_ratio, Static_best_error, 'ko');

    % Plot speed-dependent eigenmode results
    Speed_dep_Error= (SpeedDep.(results_to_plot{i})(signal_index_to_plot(i),:)-ref_result)./ref_result*100;
    Speed_dep_time_ratio= SpeedDep.t./sim_time;
    hLines(3)= plot(Speed_dep_time_ratio, Speed_dep_Error, '*-', 'Color', [0.47 0.67 0.19], 'DisplayName', 'Speed-Dependent Eigenmodes');
    % Determine the best result
    [Speed_dep_best_time_ratio, Speed_dep_best_error]= getBestResult(Speed_dep_Error, Speed_dep_time_ratio, best_error_threshold);
    plot(Speed_dep_best_time_ratio, Speed_dep_best_error, 'ko');

    % Plot Craig-Bampton results
    CB_Error= (CB.(results_to_plot{i})(signal_index_to_plot(i),:)-ref_result)./ref_result*100;
    CB_time_ratio= CB.t./sim_time;
    hLines(4)= plot(CB_time_ratio, CB_Error, 'x-', 'Color', [0.64 0.08 0.18], 'DisplayName', 'Craig-Bampton');
    % Determine the best result
    [CB_best_time_ratio, CB_best_error]= getBestResult(CB_Error, CB_time_ratio, best_error_threshold);
    plot(CB_best_time_ratio, CB_best_error, 'ko');

    ylabel(ylabels{i})
    if showLegend(i)
        legend(hLines);
    end
    if showXlabel(i)
        xlabel('Run/Sim Time Ratio');
    end

    box on;
    grid on

    %% Figure 2: Bar plots of best results
    figure(hFigure2)
    
    % Plot Best Run/Sim Time Ratio
    nexttile;
    Y1 = [Lumped_best_time_ratio, CB_best_time_ratio, Speed_dep_best_time_ratio, Static_best_time_ratio];
    b1= bar([Y1; Y1], 'FaceColor', 'Flat');
    b1(1).CData= [0 0.45 0.74; 0 0.45 0.74];       % Lumped
    b1(2).CData= [0.64 0.08 0.18; 0.64 0.08 0.18]; % CB
    b1(3).CData= [0.47 0.67 0.19; 0.47 0.67 0.19]; % Speed-Dep
    b1(4).CData= [0.93 0.69 0.13; 0.93 0.69 0.13]; % Static
    xlim([0.5 1.5])
    xticklabels('')
    ylabel('Run/Sim Time Ratio')
    title({'Best Result - Run/Sim Time Ratio'; titles{i}})
    
    % Plot Error of simulation with best Run/Sim Time Ratio
    nexttile;
    Y2 = abs([Lumped_best_error, CB_best_error, Speed_dep_best_error, Static_best_error]);
    b1= bar([Y2; Y2], 'FaceColor', 'Flat');
    b1(1).CData= [0 0.45 0.74; 0 0.45 0.74];       % Lumped
    b1(2).CData= [0.64 0.08 0.18; 0.64 0.08 0.18]; % CB
    b1(3).CData= [0.47 0.67 0.19; 0.47 0.67 0.19]; % Speed-Dep
    b1(4).CData= [0.93 0.69 0.13; 0.93 0.69 0.13]; % Static
    legend({'Lumped Mass', 'Craig-Bampton', 'Speed-Dependent Eigenmodes','Static Eigenmodes'})
    xlim([0.5 1.5])
    xticklabels('')
    ylabel('% Error')
    title({'Best Result - % Error'; titles{i}})

    % Increase figure width while holding figure left position constant
    minFigureWidth= 750; % [pixels]
    amtToIncreaseWidth= max(minFigureWidth, hFigure2.Position(3)) - hFigure2.Position(3);
    hFigure2.Position= [hFigure2.Position(1), hFigure2.Position(2),...
        hFigure2.Position(3)+amtToIncreaseWidth, hFigure2.Position(4)]; % [left, bottom, width, height]

end


end

%% Function to determine the best result of a vibration method.
% The best method is defined as the most computationally efficient
% simulation that satisfies the accuracy threshold.
function [best_time_ratio, best_error]= getBestResult(error, time_ratio, best_error_threshold)
    best_ind= find( abs(error) < best_error_threshold, 1, 'first');
    % If none of the simulations satisfy the error threshold, use the highest fidelity simulation
    if isempty(best_ind)
        best_ind= length(error);
    end
    best_time_ratio= time_ratio(best_ind);
    best_error= error(best_ind);
end