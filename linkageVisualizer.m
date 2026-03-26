% =========================================================================
% isMechanismPlanar.m
% =========================================================================
% Author:        Benjamin B Bolduc
% Last Updated:  March 2026
%
% Description: 
%   Determines whether the solver should use its 2D or 3D branch. This 
%   function checks the obtained graph geometry to see if it can be solved 
%   using the (simpler) 2D planar equations. 
% =========================================================================

function isPlanar = isMechanismPlanar(jointAxes_local, linkVectors_local, tol)
    
    % 1. The mechanism operates in the 2D XY plane (default assumption).
    isPlanar = true;
    
    % 2. Rule 1: All bodies must lie perfectly flat. We loop through the 
    % calculated vectors by the graph parser. If any vector has a Z-height 
    % greater than the selected tolerance, the mechanism is considered 3D.
    for k = 1:numel(linkVectors_local)
        if abs(linkVectors_local{k}(3)) > tol
            isPlanar = false;
            return;
        end
    end
    
    % 3. Rule 2: All joints must be parallel to the Z-axis. We loop through 
    % the defined joint axes. 
    for k = 1:numel(jointAxes_local)
        ax = jointAxes_local{k};
        
        % Ignore empty/zero axes
        if norm(ax) > 1e-9
            ax = ax / norm(ax);
            dot_z = abs(ax(3)); % Extract the Z-component of the axis
            
            % If the Z-component is not exactly 1.0 (pointing up) and not 
            % exactly 0.0 (along the floor), then the mechanism is 3D. 
            if dot_z > tol && dot_z < 1 - tol
                isPlanar = false;
                return;
            end
        end
    end
end