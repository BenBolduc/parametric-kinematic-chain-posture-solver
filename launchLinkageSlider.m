% =========================================================================
% generateAthenaHexapod.m 
% =========================================================================
% Author:        Benjamin B Bolduc
% Last Updated:  March 2026
%
% Description: 
%   Procedurally generates the JSON topology for the ATHENA Hexapod. It 
%   uses a parametric architecture to prevent human error in mapping the 
%   42 joints and 40+ rigid bodies.
%
%   Key kinematic features:
%   Ghost bodies: Zero length virtual bodies used to "stack" standard 
%       revolute joints into universal and spherical joints.
%   Virtual leg: A 6-DOF body connecting the frame directly to the platform, 
%       allowing for inverse kinematic control.
% =========================================================================

clc; clear;

%% ========================================================================
% PART 1: PARAMETRIC GEOMETRY DEFINITION
% ========================================================================
% Instead of hardcoding static coordinates, the hexapod is driven by these variables. 
R_base = 1.32;  % Radius of the base mounting ring [m]
H = 0.55;       % Default home height of the platform [m]
R_plat = 1.20;  % Radius of the moving platform ring [m]
beta = 26;      % The spacing angle between paired leg mounts [deg]

% --- Base Points (6 Distinct Anchor Nodes) ---
ang_b = deg2rad([60-(beta/2), ...
                 60+(beta/2), ...
                 180-(beta/2), ...
                 180+(beta/2), ...
                 300-(beta/2), ...
                 300+(beta/2)]);
B = [R_base*cos(ang_b); ...
     R_base*sin(ang_b); ...
     zeros(1,6)];

% --- Platform Points (3 Shared Anchor Nodes) ---
ang_p = deg2rad([60, 180, 300]);
P_3pts = [R_plat*cos(ang_p); ...
          R_plat*sin(ang_p); ...
          repmat(H, 1, 3)];

jsonStruct = struct(); 

%% ========================================================================
% PART 2: STATIC FRAME & END-EFFECTOR
% ========================================================================
% Initialize the Ground (Frame) and the End-Effector (Platform)
jsonStruct.bodies(1).name = 'Frame';

for i=1:6 
    jsonStruct.bodies(1).points(i) = struct('id', sprintf('Theta_l1_Leg%d', i), 'location', B(:,i)'); 
end

% Anchor point for the Virtual IK Leg
jsonStruct.bodies(1).points(7) = struct('id', 'Virt_X', 'location', [0,0,0]); 

jsonStruct.bodies(2).name = 'Platform';
for i=1:6 
    jsonStruct.bodies(2).points(i) = struct('id', sprintf('Theta_u3_Leg%d', i), 'location', P_3pts(:, ceil(i/2))'); 
end

% End-point for the Virtual IK Leg
jsonStruct.bodies(2).points(7) = struct('id', 'Virt_Roll', 'location', [0,0,H]); 

bodyIdx = 3; jointIdx = 1;

%% ========================================================================
% PART 3: THE 6 PHYSICAL LEGS 
% ========================================================================
for i = 1:6
    % Map Legs 1&2 to Node 1, Legs 3&4 to Node 2, Legs 5&6 to Node 3
    nodeIdx = ceil(i/2); 
    P_target = P_3pts(:, nodeIdx);
    
    % Calculate the physical vector and midpoint of the current leg
    legVec = P_target - B(:,i);
    midPt  = B(:,i) + legVec * 0.5;
    
    % --- Body naming convention ---
    name_GBU = sprintf('Ghost_Base_U%d', i); % Virtual body for universal joint
    name_L1 = sprintf('Leg_Lower%d', i); % Physical lower cylinder
    name_L2 = sprintf('Leg_Upper%d', i); % Physical upper piston
    name_GSU1 = sprintf('Ghost_Top_S1_%d', i); % Virtual body 1 for spherical joint
    name_GSU2 = sprintf('Ghost_Top_S2_%d', i); % Virtual body 2 for spherical joint
    
    % --- Body definitions ---
    % Base U-Joint (Frame -> Ghost_Base -> Leg_Lower)
    jsonStruct.bodies(bodyIdx)   = struct('name', name_GBU, 'points', [struct('id', sprintf('Theta_l1_Leg%d',i), 'location', B(:,i)'), struct('id', sprintf('Theta_l2_Leg%d',i), 'location', B(:,i)')]);
    jsonStruct.bodies(bodyIdx+1) = struct('name', name_L1, 'points', [struct('id', sprintf('Theta_l2_Leg%d',i), 'location', B(:,i)'), struct('id', sprintf('Piston%d',i), 'location', midPt')]);
    
    % Upper Leg & Top Spherical (Leg_Upper -> Ghost_S1 -> Ghost_S2 -> Platform)
    jsonStruct.bodies(bodyIdx+2) = struct('name', name_L2, 'points', [struct('id', sprintf('Piston%d',i), 'location', midPt'), struct('id', sprintf('Theta_u1_Leg%d',i), 'location', P_target')]);
    jsonStruct.bodies(bodyIdx+3) = struct('name', name_GSU1, 'points', [struct('id', sprintf('Theta_u1_Leg%d',i), 'location', P_target'), struct('id', sprintf('Theta_u2_Leg%d',i), 'location', P_target')]);
    jsonStruct.bodies(bodyIdx+4) = struct('name', name_GSU2, 'points', [struct('id', sprintf('Theta_u2_Leg%d',i), 'location', P_target'), struct('id', sprintf('Theta_u3_Leg%d',i), 'location', P_target')]);
    
    % --- Joint Definitions ---
    % Universal joint at the base (2 intersecting Revolutes)
    jsonStruct.joints(jointIdx)   = struct('label', sprintf('Theta_l1_Leg%d',i), 'bodies', {{'Frame', name_GBU}}, 'type', 'revolute', 'axis', [0,0,1]);
    jsonStruct.joints(jointIdx+1) = struct('label', sprintf('Theta_l2_Leg%d',i), 'bodies', {{name_GBU, name_L1}}, 'type', 'revolute', 'axis', [1,0,0]);
    
    % Prismatic piston in the middle
    jsonStruct.joints(jointIdx+2) = struct('label', sprintf('Piston%d',i), 'bodies', {{name_L1, name_L2}}, 'type', 'prismatic', 'axis', (legVec/norm(legVec))');
    
    % Spherical joint at the platform (3 intersecting Revolutes)
    jsonStruct.joints(jointIdx+3) = struct('label', sprintf('Theta_u1_Leg%d',i), 'bodies', {{name_L2, name_GSU1}}, 'type', 'revolute', 'axis', [0,0,1]);
    jsonStruct.joints(jointIdx+4) = struct('label', sprintf('Theta_u2_Leg%d',i), 'bodies', {{name_GSU1, name_GSU2}}, 'type', 'revolute', 'axis', [1,0,0]);
    jsonStruct.joints(jointIdx+5) = struct('label', sprintf('Theta_u3_Leg%d',i), 'bodies', {{name_GSU2, 'Platform'}}, 'type', 'revolute', 'axis', [0,1,0]);
    
    bodyIdx = bodyIdx + 5; 
    jointIdx = jointIdx + 6;
end

%% ========================================================================
% PART 4: THE 6-DOF VIRTUAL INVERSE KINEMATICS LEG
% ========================================================================
% To drive the platform directly with user inputs (X, Y, Z, Yaw, Pitch, Roll), 
% we attach a virtual 7th leg. By stacking 3 prismatic joints and 3 revolute 
% joints on top of each other at the platform's origin, fsolve is forced to 
% drag the platform to the exact coordinates requested by the user.

% Ghost bodies for the 6 degrees of freedom
jsonStruct.bodies(bodyIdx)   = struct('name', 'Virt_X_body', 'points', [struct('id', 'Virt_X', 'location', [0,0,0]), struct('id', 'Virt_Y', 'location', [0,0,0])]);
jsonStruct.bodies(bodyIdx+1) = struct('name', 'Virt_Y_body', 'points', [struct('id', 'Virt_Y', 'location', [0,0,0]), struct('id', 'Virt_Z', 'location', [0,0,0])]);
jsonStruct.bodies(bodyIdx+2) = struct('name', 'Virt_Z_body', 'points', [struct('id', 'Virt_Z', 'location', [0,0,0]), struct('id', 'Virt_Yaw', 'location', [0,0,H])]);
jsonStruct.bodies(bodyIdx+3) = struct('name', 'Virt_Yaw_body', 'points', [struct('id', 'Virt_Yaw', 'location', [0,0,H]), struct('id', 'Virt_Pitch', 'location', [0,0,H])]);
jsonStruct.bodies(bodyIdx+4) = struct('name', 'Virt_Pitch_body', 'points', [struct('id', 'Virt_Pitch', 'location', [0,0,H]), struct('id', 'Virt_Roll', 'location', [0,0,H])]);

% 3 prismatic joints
jsonStruct.joints(jointIdx)   = struct('label', 'Virt_X', 'bodies', {{'Frame', 'Virt_X_body'}}, 'type', 'prismatic', 'axis', [1,0,0]);
jsonStruct.joints(jointIdx+1) = struct('label', 'Virt_Y', 'bodies', {{'Virt_X_body', 'Virt_Y_body'}}, 'type', 'prismatic', 'axis', [0,1,0]);
jsonStruct.joints(jointIdx+2) = struct('label', 'Virt_Z', 'bodies', {{'Virt_Y_body', 'Virt_Z_body'}}, 'type', 'prismatic', 'axis', [0,0,1]);

% 3 revolute joints
jsonStruct.joints(jointIdx+3) = struct('label', 'Virt_Yaw', 'bodies', {{'Virt_Z_body', 'Virt_Yaw_body'}}, 'type', 'revolute', 'axis', [0,0,1]);
jsonStruct.joints(jointIdx+4) = struct('label', 'Virt_Pitch', 'bodies', {{'Virt_Yaw_body', 'Virt_Pitch_body'}}, 'type', 'revolute', 'axis', [0,1,0]);
jsonStruct.joints(jointIdx+5) = struct('label', 'Virt_Roll', 'bodies', {{'Virt_Pitch_body', 'Platform'}}, 'type', 'revolute', 'axis', [1,0,0]);

%% ========================================================================
% PART 5: JSON COMPILATION AND EXPORT
% ========================================================================
jsonStr = jsonencode(jsonStruct, 'PrettyPrint', true);
fid = fopen('linkages/linkage-athena.json', 'w'); 
fprintf(fid, '%s', jsonStr); 
fclose(fid);

fprintf('SUCCESS: linkage-athena.json updated to Stable Square Architecture!\n');