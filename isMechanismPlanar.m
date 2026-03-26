% =========================================================================
% closureErrors3D.m
% =========================================================================
% Author:        Benjamin B Bolduc
% Last Updated:  March 2026
%
% Description: 
%   This solver uses the principal axis and principal angle formulation. 
%   It computes local rotations, update the global orientation, and project 
%   local link vectors into the global Cartesian frame. One constraints is
%   imposed:
%   Loop closure: Going around a loop must return position error, [X, Y, Z], 
%   to [0;0;0] and orientation error, [phi, theta, psi], to [0;0;0].
% =========================================================================

function Eqs = closureErrors3D(x, mechanism, inputParams, unknownLabels)
    
    Eqs = []; % Initialize the list of errors
    
    %% ====================================================================
    % ASSEMBLING THE VARIABLES
    % ====================================================================
    % Create a complete dict of every joint state. We merge the user's 
    % input parameters with fsolve's current guesses.
    
    all_params = inputParams;

    for i = 1:numel(unknownLabels)
        all_params.(unknownLabels{i}) = x(i);
    end
    
    %% ====================================================================
    % LOOP CLOSURE EVALUATION
    % ====================================================================
    % Go across every body in the specific loop.
    for k = 1:numel(mechanism.loops)
        loop = mechanism.loops(k);
        
        % Initialize the global orientation at the start of the loop
        R = eye(3);
        
        % The first vector represents the first body
        pos = loop.vectors(:,1); 
        
        % Walk the kinematic loop
        for j = 1:(numel(loop.jointLabels)-1)
            lbl = loop.jointLabels{j};
            ax = loop.axes(:, j);
            
            % Account for graph traversal direction
            val = all_params.(lbl) * loop.dirs(j); 
            
            if loop.isPrismatic3D(j)
                R_rel = eye(3); % No rotation
                offset = ax * val; % Translate along the principal axis
            else
                R_rel = axang2rotm([ax(:)' val]); % Generate the 3x3 relative rotation matrix
                offset = [0;0;0]; % No translation
            end
            
            % Correct current global orientation
            R = R * R_rel;
            
            % Add the offset to our new global position
            pos = pos + R * (loop.vectors(:, j+1) + offset);
        end
        
        % Handle the final joint to calculate the closing orientation
        lbl = loop.jointLabels{end};
        ax = loop.axes(:, end);
        val = all_params.(lbl) * loop.dirs(end); 
        
        if loop.isPrismatic3D(end)
            pos = pos + R * (ax * val);
        else
            R_rel = axang2rotm([ax(:)' val]);
            R = R * R_rel;
        end
        
        %% ================================================================
        % COMPUTING THE RESIDUALS
        % ================================================================
        
        % 1. Position error
        pos_err = pos;
        
        % 2. Orientation error
        axang = rotm2axang(R);
        ori_err = axang(4) * axang(1:3)'; 
        
        % Stack the six errors into the equation list for fsolve
        Eqs = [Eqs; pos_err; ori_err];            
    end
end