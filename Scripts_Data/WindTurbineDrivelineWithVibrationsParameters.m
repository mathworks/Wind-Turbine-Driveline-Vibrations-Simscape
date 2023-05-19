%% Code to define parameters for WindTurbineDrivelineWithVibrations
% Open Model Workspace in the Model Explorer to view and modify parameter
% values. Click 'Reinitialize from Source' to reset to the parameter values
% in this script.

% Copyright 2022 The MathWorks, Inc.

%% Simulink configuration
fs.system_name= 'WindTurbineDrivelineWithVibrations';
fs.lumped_mass_block=   [fs.system_name '/Shaft/Lumped mass/Flexible Shaft Dynamics'];
fs.static_eig_block=    [fs.system_name '/Shaft/Static eigenmodes/Flexible Shaft Dynamics'];
fs.speed_dep_eig_block= [fs.system_name '/Shaft/Speed-dependent eigenmodes/Flexible Shaft Dynamics'];
fs.CB_block=            [fs.system_name '/Shaft/Craig-Bampton/Flexible Shaft Dynamics'];

%% Wind Scenario
scenario.tEnd = 100;
scenario.wind.Time =   [0 30  60 90 100];
scenario.wind.Values = [0 20  20 0  0];

%% Shaft length
fs.length_shaft= 10; % [m]

%% Generator unbalance transverse excitation
fs.excitationPoles= 200;    % Multiple of rotation speed
fs.excitationLoad= 20e3;    % External force magnitude [N]
fs.wNominal_forcing= 8.6/60*2*pi; % [rad/s]

%% Main numeric properties for each vibration model
fs.num_modes= 32;       % Number of modes in the static and speed-dependent eigenmode methods
fs.num_inner_modes= 14; % Number of inner modes in the Craig-Bampton method
fs.z_shaft= linspace(0, fs.length_shaft, 11); % Station locations in lumped mass model [m]
fs.w_shaft_nominal_static_eig= 4.3 *fs.excitationPoles *(2*pi/60); % Static eigenmodes nominal shaft speed [rad/s]
fs.dz= 0.2; % shaft length increments for mode shape computation in eigenmode methods [m]
fs.wMax= 1e7; % Eigenfrequency upper limit [rad/s]

%% Shaft general properties

% Diameters along the shaft
fs.z_TLU= linspace(0,fs.length_shaft,40); % Reference axial locations [m]
fs.lengths_TLU= diff(fs.z_TLU);  % Reference flexible element lengths [m]
fs.diameter_max= 1.5;   % Outer diameter at shaft B end [m]
fs.diameter_min= 1.25;  % Outer diameter at shaft F end [m]
fs.thickness_max= 0.625; % Shaft thickness at shaft B end [m]
fs.thickness_min= 0.375; % Shaft thickness at shaft F end [m]
fs.diameters_outer_TLU= linspace(fs.diameter_max, fs.diameter_min, 40); % Reference outer diameters [m]
fs.diameters_inner_TLU= linspace(fs.diameter_max-fs.thickness_min*2, fs.diameter_min-fs.thickness_max*2, 40); % Reference inner diameters [m]

% Final geometric parameters
fs.lengths= diff(fs.z_shaft); % Flexible element lengths [m]
fs.diameters_outer= interp1(fs.z_TLU, fs.diameters_outer_TLU, fs.z_shaft(2:end)); % Outer diameters [m^2]
fs.diameters_inner= interp1(fs.z_TLU, fs.diameters_inner_TLU, fs.z_shaft(2:end)); % Inner diameters [m^2]
fs.shaft_areas= pi*(fs.diameters_outer.^2 - fs.diameters_inner.^2)/4; % [m^2]

% Material properties
fs.material_density= 7.8e3; % [kg/m^3]
fs.elastic_modulus= 200e9; % [Pa]
fs.shear_modulus= 7.93e+10; % [Pa]

% Shaft material damping constants
fs.a_damp= 10;  % Damping constant proportional to mass [1/s]
fs.b_damp= 1e-9; % Damping constant proportional to stiffness [s]

% Inertia properties
fs.shaft_mass= sum( fs.material_density.*fs.lengths.*fs.shaft_areas ); % [kg]
fs.shaft_inertia= sum(fs.material_density.*fs.lengths./32.*pi.*(fs.diameters_outer.^4 - fs.diameters_inner.^4)); % [kg*m^2]
fs.F0= 9.81*fs.material_density* sum(fs.shaft_areas.*fs.lengths); % Shaft weight [N]

%% Bearing properties
fs.num_supports= 4;       % Number of bearings
fs.z_support= [0 5 8 10]; % Bearing axial locations [m] 

fs.bearing_speed= [1 2.5 5 10]; % Reference shaft speeds [rpm]

fs.mu_visc_tors= zeros(1, length(fs.z_support)); % Torsional viscosity at the supports [N*m/(rad/s)]

% Synthetic oil in bearings:
fs.bearing.oil_rho= 1077;   % Density [kg/m^3]
fs.bearing.oil_nu= 3.20e-4; % Kinematic viscosity [m^2/s]
fs.bearing.mu= fs.bearing.oil_rho * fs.bearing.oil_nu; % Dynamic viscosity [Ns/m^2]

% Bearing geometry
fs.bearing.R_meters= mean([fs.diameter_min, fs.diameter_max])/2; % Radius [m]
fs.bearing.R_inches= fs.bearing.R_meters*100/2.54;               % Radius [in]
fs.bearing.L= 0.3;                                               % Axial length [m]
fs.bearing.C= 0.1*(2*fs.bearing.R_inches);                       % Shaft-bearing diameter clearance [m]

% Bearing linearized stiffness and damping at each reference shaft speed.

% Translational stiffness [xx1, xy1, yx1, yy1;... xxS, xyS, yxS, yyS] [N/m]
% where S is the varying shaft speed.
fs.bearing.K= 1e10*[
    1.019    0.0004   -0.0112    17.882
    3.556   -0.0012   -0.0244    52.804
    6.401   -0.0022   -0.0440    79.047
    7.2011  -0.0026   -0.0488    80.928]; % [N/m]

% Translational damping   [xx1, xy1, yx1, yy1;... xxS, xyS, yxS, yyS] [Ns/m]
fs.bearing.D= 1e8*[
    2.0708   -0.0748   -0.0748    2.0708
    4.9159   -0.0256   -0.0256    4.9159
    7.4907   -0.0293   -0.0293    7.4907
    7.5885   -0.0498   -0.0498    7.5885]; % [N*s/m]

fs.bearing.k_rot= [1e2, 1e2]; % Rotational stiffness [N*m/rad]
fs.bearing.d_rot= [1e2, 1e2]; % Rotational damping [N*m*s/rad]

% Properties of optional intermediate bearing in Craig-Bampton model
fs.bearing.d_trans_speed_Free= fs.bearing.D/1e9; % [N*s/m]
fs.bearing.d_rot_Free= [1e1, 1e1];
fs.bearing.k_trans_speed_Free= fs.bearing.K/1e9;
fs.bearing.k_rot_Free= [1e1, 1e1];

%% Hub, rigid blades, and generator properties
fs.z_generator= 6;                      % Axial location of the direct drive generator [m]
fs.z_mass= [0 fs.z_generator];          % Axial location of rotor+hub and generator [m]
fs.point_mass= [8.2e4+4.9e4 1.7e5];     % Mass of rotor+hub and generator [kg]
fs.Id= [1.7e6+3.1e7 5.8e6];             % Diametric moment of inertia of rotor+hub and generator [kg*m^2]
rotor.Ip_rotor_hub= 8.6e6+6.2e7;        % Polar moment of inertia of rotor+hub [kg*m^2]
fs.Ip= [rotor.Ip_rotor_hub 7.4e6];      % Polar moment of inertia of rotor+hub and generator [kg*m^2]
fs.Ip_total= fs.shaft_inertia + sum(fs.Ip); % Total polar moment of inertis of shaft, rotor+hub and generator [kg*m^2]

%% Rotor Aerodynamic properties
rotor.R= 198/2;   % Radius [m]
rotor.rho= 1.225; % [kg/m^3]

rotor.pitch= [0     5     9    13    17    21    25];
rotor.TSR= -1:20;
rotor.Cp= [...
    0 0.0001   0.0157    0.0348    0.0950    0.2546    0.3825    0.4445    0.4738    0.4826    0.4792    0.4665    0.4475    0.4229    0.3909    0.3506    0.3031    0.2526    0.1908    0.1192    0.0375         0
    0 0.0001   0.0178    0.0433    0.1409    0.2715    0.3429    0.3681    0.3790    0.3718    0.3560    0.3331    0.2973    0.2508    0.1930    0.1234    0.0362         0         0         0         0         0
    0 0.0001   0.0196    0.0695    0.1603    0.2375    0.2584    0.2438    0.2028    0.1401    0.0584         0         0         0         0         0         0         0         0         0         0         0
    0 0.0001   0.0263    0.0790    0.1615    0.1810    0.1471    0.0793         0         0         0         0         0         0         0         0         0         0         0         0         0         0
    0 0.0001   0.0284    0.0855    0.1345    0.1052    0.0196         0         0         0         0         0         0         0         0         0         0         0         0         0         0         0
    0 0.0001   0.0294    0.0877    0.0931    0.0165         0         0         0         0         0         0         0         0         0         0         0         0         0         0         0         0
    0 0.0001   0.0299    0.0823    0.0444         0         0         0         0         0         0         0         0         0         0         0         0         0         0         0         0         0];

%% Generator torque control

% Data from Jonkman, J.; Butterfield, S.; Musial, W.; Scott, G.,
% Definition of a 5-MW reference wind turbine for offshore system development
% (No. NREL/TP-500-38060). National Renewable Energy Lab. Golden, CO, USA, 2009.
% Scaled to 10 MW wind turbine

rotor.rated_power_electric= 10; % [MW]
rotor.efficiency= 0.944;        % [-]
rotor.rated_power_mechanical= rotor.rated_power_electric/rotor.efficiency; % [MW]
rotor.TSR_opt= 8.3;            % Tip-speed ratio for peak power coefficient [-]
rotor.Cp_opt= 0.48;            % Peak power coefficient [-]
rotor.rated_wind_speed= 10.7;  % [m/s]
rotor.max_wind_speed= 25;      % [m/s]
rotor.rated_RPM= rotor.TSR_opt*rotor.rated_wind_speed/rotor.R * 60/(2*pi); % [RPM]
rotor.torque_rated= (rotor.rated_power_mechanical*10^6)/(rotor.rated_RPM*60/(2*pi)); % [N*m]
rotor.cut_in_wind_speed= 3; % [m/s]
rotor.cut_in_rpm= 4.91;     % [rpm]
rotor.drivetrain_losses_damping= rotor.rated_power_mechanical*10^6*(1-rotor.efficiency)/(rotor.rated_RPM*2*pi/60)^2; % [Nm/(rad/s)]
rotor.brake.radius= 5; % [m]

% Region 1 - cut in - parameters:
rotor.region1_start_rpm= 0;
rotor.region1_end_rpm=   rotor.cut_in_rpm;

rotor.region1_rpm=       [rotor.region1_start_rpm rotor.region1_end_rpm];
rotor.region1_torque=    [0 0]; % [N*m]

% Region 2 - optimal TSR - parameters:
rotor.region2_start_rpm= 6.38;
rotor.region2_end_rpm=   8;

rotor.region2_K= (pi*rotor.rho*rotor.R^5*rotor.Cp_opt)/(2*rotor.TSR_opt^3); % Generator torque proportionality constant [N*m/(rad/s)]
rotor.region2_rpm= linspace(rotor.region2_start_rpm, rotor.region2_end_rpm, 5);
rotor.region2_torque= rotor.region2_K*(rotor.region2_rpm*2*pi/60).^2; % [N*m]

% Region 3 - constant power - parameters:
rotor.region3_start_rpm= rotor.rated_RPM;
rotor.region3_end_rpm= 1.1*rotor.rated_RPM;

rotor.region3_rpm= linspace(rotor.region3_start_rpm, rotor.region3_end_rpm, 5);
rotor.region3_torque= (rotor.rated_power_mechanical*10^6)./(rotor.region3_rpm*2*pi/60); % [N*m]

% Final torque control lookup table, with intermediate regions 1.5 and 2.5
rotor.generator_speed_TLU=  [rotor.region1_rpm    rotor.region2_rpm     rotor.region3_rpm];
rotor.generator_torque_TLU= [rotor.region1_torque rotor.region2_torque  rotor.region3_torque]; % [N*m]

%% Rotor blade pitch control
rotor.pitch_TLU=                 [0  0.2119 0.4101 0.6084 0.8167 1.0049 1.2729 1.4911 1.7492 2.0965 2.4640 2.8811 3.3990 3.8814 4.2622 4.6685 5.1340 5.5743 6.1247 6.5566 7.0054 7.4713 7.9117 8.4454 8.8690 9.4451 10.1822 10.9193 11.5887 12.3128 12.8460 13.4347 14.1203 14.7783 15.5471 16.3299 17.0433 17.7983 18.6088 19.4124 19.9527 20.5397 21.2911 21.9424 22.4934 22.9019 95]; % [deg]
rotor.pitch_gain_correction_TLU= [1  0.9780 0.9505 0.9229 0.8978 0.8690 0.8420 0.8163 0.7918 0.7550 0.7269 0.6914 0.6570 0.6268 0.6023 0.5804 0.5543 0.5351 0.5137 0.4960 0.4783 0.4642 0.4491 0.4314 0.4210 0.4059 0.3883 0.3716 0.3576 0.3448 0.3359 0.3245 0.3130 0.3037 0.2935 0.2855 0.2766 0.2677 0.2610 0.2529 0.2487 0.2445 0.2372 0.2307 0.2250 0.2212 0.2212]; % [-]
rotor.drivetrain_I= fs.Ip_total;        % Torsional inertia [Kg*m^2]
rotor.drivetrain_naturalFrequency= 0.4; % [rad/s]
rotor.drivetrain_DampingRatio= 0.7;     % [-]
rotor.dPower_dPitch0= -2*28.24e6;       % [W/deg]
rotor.Kp0= 40*rotor.drivetrain_I*(rotor.rated_RPM*2*pi/60)*rotor.drivetrain_DampingRatio*rotor.drivetrain_naturalFrequency/-rotor.dPower_dPitch0;  % [sec]
rotor.KI0= 5*rotor.drivetrain_I*(rotor.rated_RPM*2*pi/60)*rotor.drivetrain_naturalFrequency^2/-rotor.dPower_dPitch0;                               % [-]    
rotor.pitch_Kp_TLU= rotor.Kp0*rotor.pitch_gain_correction_TLU; % Pitch control proportionality constant versus blade pitch [sec]
rotor.pitch_Ki_TLU= rotor.KI0*rotor.pitch_gain_correction_TLU; % Pitch control integral constant versus blade pitch [-]
