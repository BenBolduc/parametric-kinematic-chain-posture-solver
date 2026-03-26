% =========================================================================
% closureEqsSolver.m
% =========================================================================
% Author:        Benjamin B Bolduc
% Last Updated:  March 2026
%
% Description: 
%   Acts as the bridge between the kinematic graph and the numerical 
%   solver. It counts the unknown variables, formulates the initial 
%   guesses, and performs the Levenberg-Marquardt optimization with 
%   fsolve().
% =========================================================================

function [thetaSolved, maxResidual, posError, angError_deg, exitflag] = closureEqsSolver(thetaInputs, thetaGuesses, mechanism)
    
persistent opts_2d opts_3d
if isempty(opts_2d)
    opts_2d = optimoptions('fsolve', 'Display','none', 'Algorithm','levenberg-marquardt', ...
        'FunctionTolerance', 1e-14, 'StepTolerance', 1e-14);
    opts_3d = optimoptions('fsolve','Display','none', 'Algorithm','levenberg-marquardt');
end

    % Check the first loop to determine dimensionality
    loop1 = mechanism.loops(1);
    axesCell = num2cell(loop1.axes, 1);
    vectorsCell = num2cell(loop1.vectors, 1);
    
    if isMechanismPlanar(axesCell, vectorsCell, 1e-6)
        %% ================================================================
        % 2D PLANAR SOLVER BRANCH
        % ================================================================
        nBodies = numel(mechanism.bodies);
        movingBodyIndices = 2:nBodies; % First body is fixed and not moving
        
        Phi_guess = zeros(nBodies, 1);
        known_phi = false(nBodies, 1);
        
        % --- A. Initial guess formulation ---
        % We need a starting guess to avoid fsolve from giving us an
        % impossible configuration solution error. If we have the exact 
        % solution from the previous solver iteration we use it. Otherwise, 
        % we propagate the user's input guesses.

        if isfield(thetaGuesses, 'GlobalAngles')
            Phi_guess = thetaGuesses.GlobalAngles;
        else
            % [STEP 1] Anchor the ground. The first body is always at 0 deg
            known_phi(1) = true;

            % Merge the user's input parameters and their initial guesses 
            all_thetas = thetaGuesses;
            flds = fieldnames(thetaInputs);

            for i = 1:numel(flds)
                all_thetas.(flds{i}) = thetaInputs.(flds{i});
            end

            % [STEP 2] Forward kinematics
            usage_count = struct(); % Tracks how many times we've seen a joint name

            for pass = 1:10
                for i = 1:numel(mechanism.joints)
                    if iscell(mechanism.joints)
                        jnt = mechanism.joints{i};
                    else 
                        jnt = mechanism.joints(i); 
                    end

                    % If the user provided a guess for this joint label
                    if isfield(all_thetas, jnt.label)

                        % Find the index of the two bodies this joint connects
                        b1Idx = find(strcmp({mechanism.bodies.name}, jnt.bodies{1}));
                        b2Idx = find(strcmp({mechanism.bodies.name}, jnt.bodies{2}));

                        % [STEP 3] Propagating
                        % Checks if the global angle of one of these bodies but not the other is known
                        if (known_phi(b1Idx) && ~known_phi(b2Idx)) || (known_phi(b2Idx) && ~known_phi(b1Idx))

                            % Handles array guesses (ex: thetaGuesses.P1 = [1.9, -2.1])
                            if ~isfield(usage_count, jnt.label)
                                usage_count.(jnt.label) = 0; 
                            end

                            usage_count.(jnt.label) = usage_count.(jnt.label) + 1;
                            t_array = all_thetas.(jnt.label);

                            % If the user gave an array of guesses, use the 1st guess for the 1st
                            % joint we find, the 2nd guess for the 2nd joint, and so on...
                            idx = min(usage_count.(jnt.label), numel(t_array));
                            t_val = t_array(idx); % Local angle

                            % [STEP 4] Apply the math
                            % New global angle = known global angle +/- guessed local angle
                            if known_phi(b1Idx) && ~known_phi(b2Idx)
                                Phi_guess(b2Idx) = Phi_guess(b1Idx) + t_val;
                                known_phi(b2Idx) = true; % Mark as solved
                            else
                                Phi_guess(b1Idx) = Phi_guess(b2Idx) - t_val;
                                known_phi(b1Idx) = true; % Mark as solved
                            end
                        end
                    end
                end
            end
        end
        
        % Extract only the moving bodies for the solver to manipulate
        x0 = Phi_guess(movingBodyIndices);
        
        % --- B. Numerical Optimization ---
        % Pass the initial guesses and the graph vectors into MATLAB's fsolve.
        % We use the Levenberg-Marquardt algorithm to iteratively move and 
        % rotate every body until the sum of the vectors in every loop equals zero.
        F = @(x) closureErrors2D(x, mechanism, thetaInputs, 'atan2', movingBodyIndices, nBodies);
        [phi_sol, residual, exitflag] = fsolve(F, x0, opts_2d);
        
        % --- C. Error Extraction ---
        % fsolve returns an abstract array of mathematical errors.
        maxResidual = max(abs(residual)); 
        nLoops = numel(mechanism.loops);
        
        % 1. Position error (maximum distance between two joints)
        max_gap_m = 0;

        for k = 1:nLoops
            idx = (k-1)*2 + 1;
            gap = norm([residual(idx), residual(idx+1)]); 
            if gap > max_gap_m, max_gap_m = gap; end
        end

        posError = max_gap_m;
        
        % 2. Angular error (maximum joint angular error)
        ang_residuals = residual(2*nLoops + 1 : end);

        if isempty(ang_residuals)
            angError_deg = 0;
        else
            angError_deg = rad2deg(max(abs(ang_residuals))); 
        end
        
        % --- D. Post-Processing ---
        % The solver calculates everything in absolute global angles so we
        % need to convert these back into relative joint angles.
        thetaSolved = thetaInputs; 
        Phi_all = zeros(nBodies, 1);
        Phi_all(movingBodyIndices) = phi_sol;
        
        trueAngles = Phi_all;

        for k = 1:numel(mechanism.loops)
            loop = mechanism.loops(k);

            for b = 1:numel(loop.bodyIndices)
                if loop.isPrismatic(b)
                    trueAngles(loop.bodyIndices(b)) = 0; % Sliders do not rotate
                end
            end
        end

        thetaSolved.GlobalAngles = trueAngles;
        
        for i = 1:numel(mechanism.joints)
            if iscell(mechanism.joints)
                jnt = mechanism.joints{i}; 
            else 
                jnt = mechanism.joints(i); 
            end

            if isfield(thetaInputs, jnt.label)
                continue; 
            end

            idx1 = find(strcmp({mechanism.bodies.name}, jnt.bodies{1}));
            idx2 = find(strcmp({mechanism.bodies.name}, jnt.bodies{2}));
            val = Phi_all(idx2) - Phi_all(idx1);

            if isfield(jnt, 'type') && strcmpi(jnt.type, 'prismatic')
                thetaSolved.(jnt.label) = val; 
            else
                thetaSolved.(jnt.label) = atan2(sin(val), cos(val));
            end
        end
        
    else
        %% ================================================================
        % 3D SPATIAL SOLVER BRANCH (Multi-Loop Upgraded)
        % ================================================================
        inputLabels = fieldnames(thetaInputs);
        unknownLabels = {};
        
        % --- A. Dynamic Variable Discovery ---
        % Scan all loops to find every joint that isn't known by the user.
        for k = 1:numel(mechanism.loops)
            for j = 1:numel(mechanism.loops(k).jointLabels)
                lbl = mechanism.loops(k).jointLabels{j};
                if ~ismember(lbl, inputLabels) && ~ismember(lbl, unknownLabels)
                    unknownLabels{end+1} = lbl;
                end
            end
        end

        % Build the global guess vector dynamically
        x0_3d = zeros(numel(unknownLabels), 1);

        for i = 1:numel(unknownLabels)
            lbl = unknownLabels{i};

            if isfield(thetaGuesses, lbl)
                x0_3d(i) = thetaGuesses.(lbl);
            end
        end

        % --- B. Numerical solver ---
        F_3d = @(x) closureErrors3D(x, mechanism, thetaInputs, unknownLabels);
        [x_sol, residual, exitflag] = fsolve(F_3d, x0_3d, opts_3d);
        
        % --- C. 3D error extraction ---
        maxResidual = max(abs(residual));
        posError = 0;
        angError_deg = 0;
        nLoops = numel(mechanism.loops);
        
        % In 3D, every loop generates 6 equations (3 Translation, 3 Rotation)
        for k = 1:nLoops
            idx = (k-1)*6 + 1;
            pErr = norm(residual(idx:idx+2));
            
            aErr = rad2deg(norm(residual(idx+3:idx+5)));
            if pErr > posError
                posError = pErr; 
            end
            
            if aErr > angError_deg
                angError_deg = aErr; 
            end
        end
        
        % --- D. Post-Processing ---
        thetaSolved = thetaInputs; 

        for k = 1:numel(unknownLabels)
            thetaSolved.(unknownLabels{k}) = x_sol(k);
        end

        thetaSolved.GlobalAngles = zeros(numel(mechanism.bodies), 1);
    end
end