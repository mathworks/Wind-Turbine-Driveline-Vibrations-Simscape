function fs= WindTurbineDrivelineWithVibrationsCraigBamptonReduction(fs)
% Code to generate the Craig-Bampton reduced model of a lumped mass
% flexible shaft with transverse flexibility.
%
% The reduced model is based on a Flexible Shaft block, fs.lumped_mass_block.
% The block must have static or speed-dependent bearings.

% Copyright 2022 The MathWorks, Inc.

%% Get lumped mass model matrices

% Flag to plot modes
CBConfig.plotModes= 0;

% Obtain equation of motion matrices from the lumped mass block
assignin('base', 'fs', fs)
[num_elements, L_el_Transverse_vec, z_LumpedGBE, ...
    ~, ~, ~, ...
    M, ~, G, K, Ksupports, Dsupports, F]=...
    sdl_internal.sdl_flexible_shaft_transverse_get_lumped_properties_from_block(fs.lumped_mass_block); %#ok<ASGLU>

fs= packStruct(fs, 'num_elements', 'L_el_Transverse_vec', 'z_LumpedGBE', 'M', 'G', 'K', 'F');

evalin('base', 'clear fs')

%% Partition the matrices
% Reorder the matrices so the first DOF's are the primary DOF's (boundary DOF's)
% Original EOM:     M*q''   + (K + alpha)*q    + beta*q'  = F  - Ksupp*q - Dsupp*q' - Gw*q'   - Gw'*q
% Partitioned EOM:  Mp*qp'' + (Kp + alpha)*qp  + beta*qp' = Fp - Ksp*qp  - Dsp*qp'  - Gpw*qp' - Gpw'*qp

n= size(fs.M,1);

% Partition the degrees of freedom so the primary nodes are listed first.
% Nodes with supports and external forcing are primary nodes.
% support_nodes are in partitioned coordinates.
% primary_nodes are in original coordinates.
[Tpart, nDOF_p, ~, support_nodes, primary_nodes]= partitionEOM(fs, fs.z_LumpedGBE, n, fs.M);

% Coordinate indices
q= (1:n)';   % Original coordinates
qp= Tpart*q; %#ok<NASGU> % Partitioned coordinates

% Partitioned Mass matrix.
% Form: [Mpp, Mps    
%        Msp, Mss]
% p= primary DOF. s= secondary DOF.
Mp= Tpart * fs.M * Tpart';
Mss= Mp( nDOF_p+1:end, nDOF_p+1:end );

% Partitioned Stiffness matrix.
% Form: [Kpp, Kps    
%        Ksp, Kss]
% m= primary DOF. s= secondary DOF.
Kp= Tpart * fs.K * Tpart';
Kss= Kp( nDOF_p+1:end, nDOF_p+1:end );
Ksp= Kp( nDOF_p+1:end, 1:nDOF_p );

% Partitioned forcing matrix
Fp= Tpart * fs.F;

% Partitioned gyroscopic matrix
Gp= Tpart * fs.G * Tpart';


%% Get the static part of the CB transformation matrix.
% This is essentially the Guyan-transformation matrix.
% This matrix connects all degrees of freedom to the boundary degrees of freedom.

% Condensation matrix.
% Relates primary (q_p) and secondary (q_s) DOFs.
% q_s= Rsp * q_p
Rsp= -inv(Kss)*Ksp;

% Identity matrix
Ipp= eye(nDOF_p);

% Guyan transformation matrix.
% Columns are static modes (unit displacement of primary DOFs)
T_G= [Ipp; Rsp];

% Static part of the Craig-Bampton transformation matrix
% Relates original (q) and boundary (q_p) DOF's
% q= S_CB * q_p
S_CB= T_G;

%% Get the eigenmodes of the inner degrees of freedom.
% These are the eigenmodes when the boundary DOF's are fixed.

% Modal matrix of internal DOF's.
% Columns of Vs are fixed boundary normal modes (primary DOFs are 0)
[Vs, Ds]= eigs( sparse(Kss), sparse(Mss), fs.num_modes, 'smallestabs');
Eigenvalues= diag( Ds );
Eigenfrequencies= Eigenvalues.^.5;

p= numel(Eigenfrequencies); % number of inner modes

% Dynamic part of Craig-Bampton transformation matrix.
% Columns are eigenmodes of the inner degrees of freedom
D_CB = [zeros(nDOF_p, p); Vs];

%% Final Craig-Bampton transformation matrix
% Transforms EOM into physical coordinates at the boundaries and modal
% coordinates at inner points.

T_CB= [S_CB D_CB];
% T_CB= [ Ipp 0ps
%         Rsp Vsp ]
% Ipp- Identity matrix for primary (boundary) DOFs
% Rsp- Condensation matrix. q_s= Rsp * q_p
% Vsp- truncated secondary (inner) eigenmodes

% Plot modes to check results
if CBConfig.plotModes
    % Extract modal displacements in X and Y directions
    Modes_CB_orig_coords= Tpart\T_CB;
    CB_mode_X=     Modes_CB_orig_coords(1:4:end,:);
    CB_mode_Y=     Modes_CB_orig_coords(2:4:end,:);
    % CB_mode_theta= Modes_CB_orig_coords(3:4:end,:); % Theta, displacement about x axis
    % CB_mode_phi=   Modes_CB_orig_coords(4:4:end,:); % Phi, displacement about y axis

    % Plot static modes modes
    figure;
    numModes_static= length(S_CB(1,:));
    for i=1:numModes_static
        subplot(numModes_static,2,2*i-1)
        plot(z_LumpedGBE, CB_mode_X(:,i)./max( abs(CB_mode_X(:,i)) ));
        title(['Static CB Mode ' num2str(i) ' X'])

        subplot(numModes_static,2,2*i)
        plot(z_LumpedGBE, CB_mode_Y(:,i)./max( abs(CB_mode_Y(:,i)) ) );
        title(['Static CB Mode ' num2str(i) ' Y'])
    end

    % Plot dynamic modes
    figure;
    numModes_inner= length(D_CB(1,:));
    for i=1:numModes_inner
        j= i + numModes_static;
        subplot(numModes_inner,2,2*i-1)
        plot(z_LumpedGBE, CB_mode_X(:,j)./max( abs(CB_mode_X(:,j)) ));
        title(['Inner CB Mode ' num2str(i) ' X'])

        subplot(numModes_inner,2,2*i)
        plot(z_LumpedGBE, CB_mode_Y(:,j)./max( abs(CB_mode_Y(:,j)) ) );
        title(['Inner CB Mode ' num2str(i) ' Y'])
    end
end

%% Apply the CB transformation to the EOM matrices
% Original EOM:     M*q''   + (K + alpha)*q    + beta*q'  = F  - Ksupp*q - Dsupp*q' - Gw*q'   - Gw'*q
% Partitioned EOM:  Mp*qp'' + (Kp + alpha)*qp  + beta*qp' = Fp - Ksp*qp  - Dsp*qp'  - Gpw*qp' - Gpw'*qp
% CB EOM:           Mc*qc'' + (Kc + alpha)*qc  + beta*qc' = Fc - Ksc*qc  - Dsc*qc'  - Gcw*qc' - Gcw'*qc

Mc= T_CB'*Mp*T_CB;
Kc= T_CB'*Kp*T_CB;
Fc= T_CB'*Fp;
Gc= T_CB'*Gp*T_CB;

% Normalized mass matrix
nDOF_CB= size( Mc, 1);
norm_M= inv( Mc );

% Normalize the CB EOM terms by Mc
M_CB= eye(nDOF_CB);
K_CB= Mc\Kc;
F_CB= Mc\Fc;
G_CB= Mc\Gc;

% Matrices in CB EOM
fs.M= M_CB;
fs.normM= norm_M;
fs.G= G_CB;
fs.K= K_CB;
fs.F= F_CB;
fs.support_nodes= support_nodes;
fs.num_primary_nodes= length(primary_nodes);
fs.num_CB_modes= p + nDOF_p;

% Extract modal displacements in X and Y directions
Modes_CB_orig_coords= Tpart\T_CB;
fs.CB_mode_X= Modes_CB_orig_coords(1:4:end,:);
fs.CB_mode_Y= Modes_CB_orig_coords(2:4:end,:);
fs.CB_mode_z= z_LumpedGBE;

end

%% Helper functions

%% -------------------------------------------------------------------------
function [Tpart, nDOF_m, num_secondary_segments, support_nodes, primary_nodes]= partitionEOM(fs, z_LumpedGBE, n, M)
% Partition the degrees of freedom so the primary nodes are listed first.
% Nodes with supports and external forcing are primary nodes.

    z_support= fs.z_support;
    z_offset= fs.z_generator;
    [z_primary, primary_nodes, ~]= intersect(z_LumpedGBE, [ z_support z_offset]);

    % support_node are indices in the partitioned matrix of supports
    [~, support_nodes, ~]= intersect(z_primary, z_support);

    num_nodes= length(z_LumpedGBE);
    nodes= 1:num_nodes;
    secondary_nodes= nodes( ~ismember(nodes,primary_nodes) );
    num_secondary_segments= 1;
    for j= 1:length(secondary_nodes)-1
        if ( secondary_nodes(j + 1) ~= secondary_nodes(j) + 1 )
            num_secondary_segments= num_secondary_segments + 1;
        end
    end

    % Coordinates to swap:
    % Each node has 4 degrees of freedom
    % Primary indices are the interface indices.
    % They are where supports and forcing occur.
    num_primary_nodes= numel( primary_nodes );
    
    primary_inds= zeros(1, 4*num_primary_nodes);
    for j= 1:num_primary_nodes
        primary_inds(  4*j-3 : 4*j )= 4*primary_nodes(j) + [-3 -2 -1 0];
    end
    
    all_inds= 1:n;
    [~,internal_inds,~]= setxor(all_inds, primary_inds);
    internal_inds= internal_inds'; % row vector
    
    % Partitioned coordinates [m; s] are q_p= q
    n= size(M,1);
    S_init= eye(n);
    
    nDOF_m= 4*num_primary_nodes;
    
    % Let q be the orignal DOF vector.
    % q_p is the partitioned DOF vector
    % Ssr swaps rows
    Ssr= [ S_init(primary_inds,:); S_init(internal_inds,:) ];
    % Ssc swaps columns
    Ssc= Ssr'; %#ok<NASGU> 
    
    % Let Tpart= Ssr
    Tpart= Ssr;

end

%% ------------------------------------------------------------------------
function S= packStruct(S, varargin)
% Assign a list of inputs, varargin, to a struct, S

    params = varargin;
    for i= 1:length(params)
          S.(params{i})= evalin('caller', params{i});
    end

end
