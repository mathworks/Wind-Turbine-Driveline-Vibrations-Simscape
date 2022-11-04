% Code to plot the bending modes from the WindTurbineDrivelineWithVibrations demo
%% Plot Description:
%
% This figure shows the bending mode shapes of a shaft in its two
% transverse directions with parameters specified in the 
% Speed-dependent eigenmodes block of the
% WindTurbineDrivelineWithVibrations model.
% The transverse motion is excited by an external force with a frequency
% equal to fs.excitationPoles times the shaft spin speed.

% Copyright 2018-2022 The MathWorks, Inc.

% Inputs for mode calculation
hModes.blockHandle= getSimulinkBlockHandle('WindTurbineDrivelineWithVibrations/Shaft/Speed-dependent eigenmodes/Flexible Shaft Dynamics');

% Reuse figure if it exists, else create new figure
if ~exist('h3_WindTurbineDrivelineWithVibrations', 'var') || ...
        ~isgraphics(h3_WindTurbineDrivelineWithVibrations, 'figure')
    h3_WindTurbineDrivelineWithVibrations= figure('Name', 'Shaft Bending Vibration');
end
figure(h3_WindTurbineDrivelineWithVibrations)
clf(h3_WindTurbineDrivelineWithVibrations)

% Temporarily add fs parameter structure to base workspace if it does not exist
if ~exist('fs', 'var')
    fsExistsInBaseWS= 0;
    hws= get_param('WindTurbineDrivelineWithVibrations', 'modelworkspace');
    fs= getVariable(hws, 'fs');
    assignin('base', 'fs', fs);
else
    fsExistsInBaseWS= 1;
end

hModes.hX= subplot(2,2,1);
hModes.hX.Position= [0.1 0.6 0.6 0.32];
box on
hold on
title('Mode Shapes in X Direction');
ylabel('U_X');
xlabel('Shaft position (m)');

hModes.hY= subplot(2,2,3);
hModes.hY.Position= [0.1 0.1 0.6 0.32];
box on
hold on
title('Mode Shapes in Y Direction');
ylabel('U_Y');
xlabel('Shaft position (m)');
hModes.legend= legend();
hModes.legend.Position= [ 0.66 0.22 0.137 0.043 ];

hModes.hCalc= annotation('textbox',...
       'Units', 'normalized', 'Position', [0.38,0.56,0.15,0.06],...
       'FontSize', 16, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle',...
       'String','Calculating...', 'backgroundcolor', 'none', 'LineStyle', 'none',...
        'visible', 'off');

% Choose initial shaft rotation speed [RPM]:
hModes.OmegaNom_RPM= 8;

% User interface
annotation('textbox',...
   'Units', 'normalized', 'Position',[0.7 0.7 0.15 0.06],...
   'FontSize', 8, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle',...
   'String','Shaft rotation speed (RPM):', 'backgroundcolor', 'none', 'LineStyle', 'none');

hModes.hText = uicontrol('Style', 'edit', 'String', num2str(hModes.OmegaNom_RPM, '%.0f'), ...
    'Units', 'normalized', 'Position', [0.86 0.7 0.1 0.05]);

annotation('textbox',...
   'Units', 'normalized', 'Position',[0.76 0.64 0.16 0.045],...
   'FontSize', 8, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle',...
   'String','Modes to plot:', 'backgroundcolor', 'none', 'LineStyle', 'none');

hModes.hText_mode = uicontrol('Style', 'listbox', ...
    'Min', 1, 'Units', 'normalized', 'Position', [0.76 0.38 0.17 0.26]);

hModes.hPlot= uicontrol('Style', 'pushbutton', 'String', 'Plot', ...
    'Units', 'normalized', 'Position', [0.73 0.79 0.1 0.05], ... 
    'Callback', {@plot_modes, hModes} );

uicontrol('Style', 'pushbutton', 'String', 'Clear', ...
    'Units', 'normalized', 'Position', [0.86 0.79 0.1 0.05], ...
    'Callback', {@clear_axes, h3_WindTurbineDrivelineWithVibrations});

hModes.h_calculate= uicontrol('Style', 'pushbutton', 'String', 'Calculate modes', ...
    'Units', 'normalized', 'Position', [0.73 0.87 0.23 0.05], ...
    'Callback', {@calculate_modes, hModes, h3_WindTurbineDrivelineWithVibrations});

if ~exist( 'flex_shaft_mode_props', 'var' )
    calculate_modes([],[], hModes, h3_WindTurbineDrivelineWithVibrations);
    hModes.hText_mode.Value= 1 : min(4, hModes.hText_mode.Max);
    plot_modes([], [], hModes)
end

if ~fsExistsInBaseWS
    clear fs hws
end
clear fsExistsInBaseWS

%% --- Subfunctions ---
function calculate_modes(~, ~, hModes, hFigure)

    set(hModes.hCalc, 'visible', 'on')
    set(hModes.hX, 'visible', 'off')
    set(hModes.hY, 'visible', 'off')
    clear_axes([], [], hFigure)
    drawnow
    
    % Update block parameters related to num_poles harmonic forcing
    hws= get_param('WindTurbineDrivelineWithVibrations', 'modelworkspace');
    fs= getVariable(hws, 'fs');
    fs.excitationPoles= 1;
    assignin('base', 'fs', fs);

    % Get mode shape arrays with the format [zshaft, mode, shaft_speed]
    [WX, WY, zshaft, eigenfrequencies, shaft_speeds]=...
    sdl_internal.sdl_flexible_shaft_transverse_get_modes_from_block(hModes.blockHandle, [], []);

    set(hModes.hCalc, 'visible', 'off')
    set(hModes.hX, 'visible', 'on')
    set(hModes.hY, 'visible', 'on')
    
    % Store properties in a structure
    flex_shaft_mode_props.WX= WX;
    flex_shaft_mode_props.WY= WY;
    flex_shaft_mode_props.zshaft= zshaft;
    flex_shaft_mode_props.modeFreqs= eigenfrequencies;
    flex_shaft_mode_props.shaft_speeds_RPM= shaft_speeds * 60 / (2*pi);

    assignin('base', 'flex_shaft_mode_props', flex_shaft_mode_props);

    % Update listbox of mode #'s
    hModes.hText_mode.String= num2str( (1: size(flex_shaft_mode_props.modeFreqs,2) )' );
    hModes.hText_mode.Max= size(flex_shaft_mode_props.modeFreqs,2);
    hModes.hText_mode.Value= hModes.hText_mode.Value( hModes.hText_mode.Value <=  hModes.hText_mode.Max); % saturate selected values
    
end

function plot_modes( ~, ~, hModes)

    OmegaNom_RPM= str2double(hModes.hText.String);
    props= evalin('base', 'flex_shaft_mode_props');

    if length( props.shaft_speeds_RPM ) > 1
        % saturate mode properties at input data bounds, as done during simulation
        shaft_speed_lookup= min( max( props.shaft_speeds_RPM(1), OmegaNom_RPM ), props.shaft_speeds_RPM(end) ); 

        % Determine mode shape at the specified rotation speed
        % WX has format [zshaft, mode, shaft_speed]. WX_plot has format [zshaft, mode]
        % interp1 interpolates along columns
        WX_plot= permute( interp1(props.shaft_speeds_RPM, permute(props.WX, [3,1,2]), shaft_speed_lookup), [2,3,1]);
        WY_plot= permute( interp1(props.shaft_speeds_RPM, permute(props.WY, [3,1,2]), shaft_speed_lookup), [2,3,1]);
    else
        % Calculated mode shapes do not depend on rotation speed
        WX_plot= props.WX;
        WY_plot= props.WY;
    end

    if ~isempty(props.modeFreqs)

        if length ( props.shaft_speeds_RPM ) > 1
            mode_frequency_plot= interp1(props.shaft_speeds_RPM, props.modeFreqs, shaft_speed_lookup);
        else
            mode_frequency_plot = props.modeFreqs;
        end

        for i= hModes.hText_mode.Value
            line_label = [ 'w' num2str(i) '= ',...
            num2str( mode_frequency_plot(i)*60/(2*pi), '%.0f'  ) ' RPM. ',...
            'Shaft speed= ' num2str( OmegaNom_RPM ) ' RPM' ];
            plot(hModes.hX, props.zshaft, WX_plot(:,i), 'DisplayName', line_label );
        end
        for i= hModes.hText_mode.Value
            line_label = [ 'w' num2str(i) '= ',...
            num2str( mode_frequency_plot(i)*60/(2*pi), '%.0f'  ) ' RPM. ',...
            'Shaft speed= ' num2str( OmegaNom_RPM ) ' RPM' ];
            plot(hModes.hY, props.zshaft, WY_plot(:,i), 'DisplayName', line_label );
        end
    end
   
end

function clear_axes(~, ~, hModes)
    allaxes = findall(hModes, 'type', 'axes');
    hX= allaxes(1);
    hY= allaxes(2);

    delete(hX.Children)
    delete(hY.Children)
    
end

