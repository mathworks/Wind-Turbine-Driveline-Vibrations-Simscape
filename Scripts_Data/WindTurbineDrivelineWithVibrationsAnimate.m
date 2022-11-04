% WindTurbineDrivelineWithVibrationsAnimate
% Code to animate transverse vibrations from WindTurbineWithVibrations model.
%
% This script generates a figure that shows an animation of the bending
% deflection of a rotating shaft in its two transverse directions over time.
% The transverse deflection can be modeled by lumped masses, eigenmodes, or
% the Craig-Bampton methods. Positive shaft rotation in this animation
% corresponds to counter-clockwise rotation about the Z-axis, which points
% from the shaft B node toward the F node. The red circles correspond to
% concentrated inertias on the shaft.

% Copyright 2018-2022 The MathWorks, Inc.

% Generate simulation results if they don't exist
if ~exist('WindTurbineOut', 'var') || isempty(WindTurbineOut.simlog) || pm_simlogNeedsUpdate(WindTurbineOut.simlog)
    WindTurbineOut= sim('WindTurbineDrivelineWithVibrations');
end

% Reuse figure if it exists, else create new figure
if ~exist('h2_WindTurbineDrivelineWithVibrations', 'var') || ...
        ~isgraphics(h2_WindTurbineDrivelineWithVibrations, 'figure')
    h2_WindTurbineDrivelineWithVibrations= figure('Name', 'Shaft Bending Vibration');
end
figure(h2_WindTurbineDrivelineWithVibrations)
clf(h2_WindTurbineDrivelineWithVibrations)

% Temporarily add fs parameter structure to base workspace if it does not exist
if ~exist('fs', 'var')
    fsExistsInBaseWS= 0;
    hws= get_param('WindTurbineDrivelineWithVibrations', 'modelworkspace');
    fs= getVariable(hws, 'fs');
    assignin('base', 'fs', fs);
else
    fsExistsInBaseWS= 1;
end

% Get block parameters and logged response
blkPath= 'WindTurbineDrivelineWithVibrations/Shaft';
vibration_model= get_param(blkPath, 'vibration_model');
if strcmp(vibration_model, 'Lumped mass')
    [t, z, D, zDisk, D_Disk]= getLumpedResponse([blkPath '/Lumped mass/Flexible Shaft Dynamics'], WindTurbineOut.simlog.Shaft.Lumped_mass.Flexible_Shaft_Dynamics.fsBendGBE);
elseif strcmp(vibration_model, 'Static eigenmodes')
    [t, z, D, zDisk, D_Disk]= getEigenmodeResponse([blkPath '/Static eigenmodes/Flexible Shaft Dynamics'], WindTurbineOut.simlog.Shaft.Static_eigenmodes.Flexible_Shaft_Dynamics.fsBendEigen);
elseif strcmp(vibration_model, 'Speed-dependent eigenmodes')
    [t, z, D, zDisk, D_Disk]= getEigenmodeResponse([blkPath '/Speed-dependent eigenmodes/Flexible Shaft Dynamics'], WindTurbineOut.simlog.Shaft.Speed_dependent_eigenmodes.Flexible_Shaft_Dynamics.fsBendEigen);
elseif strcmp(vibration_model, 'Craig-Bampton')
    [t, z, D, zDisk, D_Disk]= getCraigBamptonResponse(WindTurbineOut.simlog.Shaft.Craig_Bampton.Flexible_Shaft_Dynamics, fs, eval(get_param(blkPath, 'num_modes_CB')));
end

if ~strcmp(vibration_model, 'Rigid')
    mainAnimation(t, z, D, zDisk, D_Disk, h2_WindTurbineDrivelineWithVibrations);
end

if ~fsExistsInBaseWS
    clear fs hws
end
clear fsExistsInBaseWS blkPath vibration_model t z D zDisk D_Disk


%% ------------------------------------------------------------------------
function [t, z_LumpedGBE, D, z_Disk, D_Disk] = getLumpedResponse(blkPath, shaft_simlog)

  
      [num_elements, L_el_Transverse_vec, z_LumpedGBE, ...
        Itors_Global, Dtors_Global, Ktors_Global, ...
        MGlobal, MGlobalAlpha, GGlobal, KGlobal, Ksupports, Dsupports, FGlobal]=...
        sdl_internal.sdl_flexible_shaft_transverse_get_lumped_properties_from_block(blkPath); %#ok<*ASGLU>
    
    num_stations= num_elements+1;
    
    % Simulation data
    t = shaft_simlog.Vxsense.series.time';
    % Element transverse velocities in X direction Each column is a different node:
    Vx= shaft_simlog.Vxsense.series.values('m/s');
    % Element transverse velocities in Y direction Each column is a different node:
    Vy= shaft_simlog.Vysense.series.values('m/s');
    V= Vx + Vy.*1i;
    
    D= cumtrapz(t, V); % displacement
    
    D= D(:,1:num_stations);
    
    % Rigid masses or "disks" that are attached to shaft
    no_Disk= evalin('base', get_param(blkPath, 'noDisk')); % Flag indicating if shaft has disks
    z_Disk= evalin('base', get_param(blkPath, 'zDisk')); % Assume units of [m]
    
    if no_Disk ~= 1
        %Disk deflection versus time
        D_Disk= interp1(z_LumpedGBE', D', z_Disk);
        D_Disk= D_Disk';
    else
        D_Disk= [];
    end

end

%% ------------------------------------------------------------------------
function [t, z, D, z_Disk, D_Disk] = getEigenmodeResponse(blkPath, shaft_simlog)
    
    parameterization= evalin('base', get_param(blkPath, 'parameterization'));
    if parameterization <= 2 % Block uses scalar parameter for shaft length
        shaft_length= evalin('base', get_param(blkPath, 'length')); % assume units of [m]
    else % Block uses vector parameter for shaft segment lengths
        shaft_length= sum( evalin('base', get_param(blkPath, 'lengths')) ); % assume units of [m]
    end
    numPts= 100; %number of points to plot along shaft axis
    z= linspace(0, shaft_length, numPts); %shaft locations to plot in animation
    
    % Get mode shapes at each reference shaft speed.
    % mode shape arrays have the format [shaft_speed, z_shaft, mode]
    [UX_tab, UY_tab,~, ~, shaft_speed_tab]= sdl_internal.sdl_flexible_shaft_transverse_get_modes_from_block(blkPath, z, []);
    numSpeeds_computed= size(shaft_speed_tab, 2);
    numModes_computed= size(UX_tab,2);

    UX_tab= UX_tab(:,1:numModes_computed,:);
    UY_tab= UY_tab(:,1:numModes_computed,:);

    % Simulation data
    t = shaft_simlog.eta.series.time;
    shaft_speed= shaft_simlog.shaft_w.series.values('rad/s');

    %  Deta, mode time-dependent coefficient displacement of real and imaginary components of the modes
    % format: [real components; imaginary components]
    Deta= shaft_simlog.eta.series.values('1');

    % Animation frame data based on geometry and simulation data
    Dx= zeros(numel(t), numel(z));
    Dy= zeros(numel(t), numel(z));
    
    for j= 1:numel(t)
    
      % time components of the modes
      eta= Deta(j,:); 
    
      %expand time component to every position along shaft
      etaMat= repmat(eta, numel(z),1 ); 
    
      if numSpeeds_computed >1
         shaft_speed_lookup= min( max( shaft_speed_tab(1),shaft_speed(j) ), shaft_speed_tab(end) ); % saturate mode properties at input data bounds
    
         % Determine mode shape at the specified rotation speed
         % WX has format [zshaft, mode, shaft_speed]. WX_plot has format [zshaft, mode]
         % interp1 interpolates along columns
         UX_inst= permute( interp1(shaft_speed_tab, permute(UX_tab, [3,1,2]), shaft_speed_lookup), [2,3,1]);
         UY_inst= permute( interp1(shaft_speed_tab, permute(UY_tab, [3,1,2]), shaft_speed_lookup), [2,3,1]);
      else % constant mode properties
         UX_inst= UX_tab;
         UY_inst= UY_tab; 
      end
    
      %Sum column-wise to get final instantaneous displacement of shaft at each position.
      Dx(j,:)=  sum( real( UX_inst'.*etaMat') , 1); 
      Dy(j,:)=  sum( real( UY_inst'.*etaMat') , 1);
    
    end
    
    D= Dx + 1i.*Dy;
    
    % Rigid masses or "disks" that are attached to shaft
    no_Disk= evalin('base', get_param(blkPath, 'noDisk')); % Flag indicating if shaft has disks
    z_Disk= evalin('base', get_param(blkPath, 'zDisk')); % Assume units of [m]
    
    if no_Disk ~= 1
        %Disk deflection versus time
        D_Disk= interp1(z', D', z_Disk);
        D_Disk= D_Disk';
    else
        D_Disk= [];
        z_Disk= [];
    end

end

%% ------------------------------------------------------------------------
function [t, z, D, z_Disk, D_Disk] = getCraigBamptonResponse(shaft_simlog, fs, num_modes)

    % Get mode shapes at each reference shaft speed.
    fs.ne_Lumped= 50;
    fs.z_shaft= linspace(0, fs.length_shaft, fs.ne_Lumped);
    assignin('base', 'fs', fs);
    fsCB= fs;
    fsCB.num_modes= num_modes;
    fsCB= WindTurbineDrivelineWithVibrationsCraigBamptonReduction(fsCB);

    % Each column is a mode shape. Each row is a different node
    UX= fsCB.CB_mode_X;
    UY= fsCB.CB_mode_Y;
    z= fsCB.CB_mode_z; % assumed units [m]

    % Simulation data
    t = shaft_simlog.eta.series.time;

    % Deta, mode time-dependent coefficient displacement of modes
    % Each row is a different time step. Each column is a different mode.
    Deta= shaft_simlog.eta.series.values('1');

    % Animation frame data based on geometry and simulation data
    Dx= zeros(numel(t), numel(z));
    Dy= zeros(numel(t), numel(z));
    
    for j= 1:numel(t)
    
      % time components of the modes
      eta= Deta(j,:); 
    
      % expand time component to every node along shaft
      etaMat= repmat(eta, numel(z), 1); 

      % Sum column-wise to get final instantaneous displacement of shaft at each position.
      Dx(j,:)=  sum( UX'.*etaMat', 1); 
      Dy(j,:)=  sum( UY'.*etaMat', 1);
    
    end
    
    % D: each row is a different time step. Each column is a different node.
    % Real component is displacement in X. Imaginary component is displacement in Y
    D= Dx + 1i.*Dy;
    
    % displacement of rigid disks that are attached to shaft
    z_Disk= fs.z_mass; % Assume units of [m]
    D_Disk= interp1(z', D', z_Disk);
    D_Disk= D_Disk';

end

%% ------------------------------------------------------------------------
function mainAnimation(t, z, D, zDisk, D_Disk, hFigure)
% Animates the transverse motion of the flexible shaft.

% Copyright 2018 The MathWorks, Inc.

% Interpolate results to obtain equal time steps
dt= 0.01; %[s]
t_mov = t(1):dt:t(end);
ind_start= round(length(t_mov)/3);
t_start= t_mov(ind_start);
d_x = real(interp1(t, D, t_mov)).*1000; %UNITS: mm
d_y = imag(interp1(t, D, t_mov)).*1000; %UNITS: mm

numDisks= length(zDisk);
if ~isempty(D_Disk)
    disk_x= real(interp1(t, D_Disk, t_mov)).*1000; %UNITS: mm
    disk_y= imag(interp1(t, D_Disk, t_mov)).*1000; %UNITS: mm

    %if there is only 1 disk, make sure disk_cycle is a column vector
    if numDisks==1
        if ~iscolumn(disk_x)
            disk_x= disk_x';
            disk_y= disk_y';
        end
    end
end

clf(hFigure)
box on

% Plot shaft positions at t = t_start
hold on
handle_shaft = plot3(d_x(ind_start,:), z, d_y(ind_start,:), '-o', 'color', [0.5 0.5 0.5], 'LineWidth', 1, 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerSize', 6);

%Plot disks
if numDisks>0
    handle_disk = plot3(disk_x(ind_start,:), zDisk, disk_y(ind_start,:), 'o', 'color', [0.15 0.15 0.15], 'MarkerFaceColor', [0.15 0.15 0.15], 'MarkerSize', 20);
end
hold off
minY= min(min(d_y));
maxY= max(max(d_y));
minX= min(min(d_x));
maxX= max(max(d_x));
if maxY > minY
  set(gca, 'ZLim', [minY, maxY].*1.1)
  set(gca, 'XLim', [minX, maxX].*1.1)
end
Ytext= maxY/5;
text(0,0,Ytext, 'B','FontSize', 16, 'Color', [0.47,0.67,0.19])
text(0,z(end),Ytext,'F','FontSize', 16, 'Color', [0.47,0.67,0.19])

xlabel('X (mm)')
zlabel('Y (mm)')
ylabel('Axial position (m)')
view([-39.8 29.8]);
grid on
titleHandle = title(sprintf('Transverse Vibration Animation (t = %.4f s)', t_start));

% Create Play/Pause button for animation.
status = 'paused';
iPaused = 1;
i= 1;
hButton = uicontrol('Style', 'pushbutton', 'String', 'Start', ...
    'Units', 'normalized', 'Position', [0.65 0.85, 0.1, 0.05], ...
    'Callback', @(hObject, eventData) playAnimation);

uicontrol('Style', 'pushbutton', 'String', 'Restart', ...
    'Units', 'normalized', 'Position', [0.77 0.85, 0.1, 0.05], ...
    'Callback', @(hObject, eventData) restartTime);

hText_dt = uicontrol('Style', 'edit', 'String', num2str(dt, '%.1e'), ...
    'Units', 'normalized', 'Position', [0.77 0.78, 0.1, 0.05], ...
    'Callback', @(hObject, eventData) adjustTimeSteps);

annotation('textbox',...
        'Units', 'normalized', 'Position',[0.57 0.78, 0.18, 0.05],...
        'FontSize', 8, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle',...
        'String','Time step, dt (s):', 'backgroundcolor', [1 1 1]);


    function adjustTimeSteps
        % Interpolate results to obtain new time steps
        dt= str2double(hText_dt.String);
        currentTime= t_mov(i);
        t_mov = currentTime:dt:t(end);
       
        d_x = real(interp1(t, D, t_mov)).*1000; %UNITS: mm
        d_y = imag(interp1(t, D, t_mov)).*1000; %UNITS: mm
        if ~isempty(D_Disk)
            disk_x= real(interp1(t, D_Disk, t_mov)).*1000; %UNITS: mm
            disk_y= imag(interp1(t, D_Disk, t_mov)).*1000; %UNITS: mm
        end
        if numDisks==1
            if ~iscolumn(disk_x)
                disk_x= disk_x';
                disk_y= disk_y';
            end
        end

        %Adjust iPaused- assume continuing not restarting
        iPaused= 1;
    end

    function restartTime
        % Reset time index
        iPaused= 1; %new index
        i= 1;
        t_mov= t;
        adjustTimeSteps;
        playAnimation;
    end


    function playAnimation
        % Nested function to loop through time and set the deflection point data
        try
            if strcmp(status, 'playing')
                status = 'paused';
                set(hButton, 'String', 'Continue')
                return
            end
            
            status = 'playing';
            set(hButton, 'String', 'Pause')
            
            % Plot transverse displacement along shaft
            for i = iPaused : length(t_mov) %#ok<FXUP>
                if strcmp(status, 'paused')
                    % Save state of the animation.
                    iPaused = i;
                    return
                end
                set(handle_shaft, 'XData', d_x(i,:))
                set(handle_shaft, 'ZData', d_y(i,:))
                if numDisks>0
                    set(handle_disk, 'XData', disk_x(i,:))
                    set(handle_disk, 'ZData', disk_y(i,:))
                end
                
                
                set(titleHandle, 'String', sprintf('Transverse Vibration Animation (t = %.3f s)', t_mov(i)) )
                drawnow
            end
            
            status = 'paused';
            set(hButton, 'String', 'Play')
            iPaused = 1;
            
        catch ME
            % End gracefully if user closed figure during the animation.
            if ~strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
                rethrow(ME)
            end
        end
    end
end
