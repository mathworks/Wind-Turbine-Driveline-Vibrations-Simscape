% Code to plot the Campbell diagram for the WindTurbineDrivelineWithVibrations demo
%% Plot Description:
%
% This figure shows the Campbell frequency diagram for the bending
% of a shaft rotating at varied speeds. The shaft is specified
% the flexShaft_eigen block of the sdl_flexible_shaft_transverse model.

% Copyright 2022 The MathWorks, Inc.

% Reuse figure if it exists, else create new figure
if ~exist('h4_WindTurbineDrivelineWithVibrations', 'var') || ...
        ~isgraphics(h4_WindTurbineDrivelineWithVibrations, 'figure')
    h4_WindTurbineDrivelineWithVibrations = figure('Name', 'h4_WindTurbineDrivelineWithVibrations');
end

plotCampbellDiagram(h4_WindTurbineDrivelineWithVibrations,...
    'WindTurbineDrivelineWithVibrations/Shaft/Speed-dependent eigenmodes/Flexible Shaft Dynamics');

function plotCampbellDiagram(hFigure, hBlock)

    figure(hFigure);
    clf(hFigure);
    hold on

    % Get frequencies
    hws= get_param('WindTurbineDrivelineWithVibrations', 'modelworkspace');
    fs= getVariable(hws, 'fs');
    assignin('base', 'fs', fs);
    bearing_speeds= fs.bearing_speed; % [rpm]
    
    [~, ~, ~, eigenfrequencies, ~]= sdl_internal.sdl_flexible_shaft_transverse_get_modes_from_block(hBlock, [], [] );

    % Plot eigenfrequencies
    for k=1:size(eigenfrequencies, 2)
        plot( bearing_speeds, eigenfrequencies(:, k)./(2*pi).*60, '+-', 'DisplayName', ['Mode ' num2str(k)]);
    end
    
    % Plot line of 200p frequencies
    plot(linspace(0,1,100).*bearing_speeds(end),linspace(0,200,100).*bearing_speeds(end), '--r', 'LineWidth', 1, 'DisplayName', '200x shaft speed')
    
    hLegend= legend;
    hLegend.Location= 'eastoutside';
    xlim([bearing_speeds(1) bearing_speeds(end)]);
    box on
    ylabel('Whirling frequencies (RPM)');
    xlabel('Shaft speed (RPM)');
    title('Campbell Diagram')
    set(gca, 'YScale', 'log')
    
    % Increase figure height, while holding figure top position constant
    minFigureHeight= 550; % [pixels]
    amtToIncreaseHeight= max(minFigureHeight, hFigure.Position(4)) - hFigure.Position(4);
    hFigure.Position= [hFigure.Position(1), hFigure.Position(2)-amtToIncreaseHeight,...
                       hFigure.Position(3), hFigure.Position(4)+amtToIncreaseHeight]; % [left, bottom, width, height]

end
