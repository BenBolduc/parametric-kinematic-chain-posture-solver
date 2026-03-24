% =========================================================================
% solvePosKin.m
% =========================================================================
% Author:        Benjamin B Bolduc
% Last updated:  March 2026
%
% Description: 
%   This function serves as the JSON parser for the solver. It is fed an
%   abstract JSON mechanism definition and applies graph theory to 
%   autonomously find independent kinematic loops. It then extracts local 
%   vectors across each body, setting up the geometry required for the 
%   numerical loop-closure equations.
% =========================================================================

function mechanism = solvePosKin(jsonFile)
    
    %% ====================================================================
    % PART 1: GEOMETRY PARSING (Abstract Data to MATLAB Struct)
    % ====================================================================
    % The simulation is initialized by loading the JSON file and converting 
    % it into a MATLAB struct. Extract the spatial coordinates [X, Y, Z] 
    % for every connection point on every mechanical body.

    jsonData = jsondecode(fileread(jsonFile));
    nB = numel(jsonData.bodies);
    mechanism.bodies = struct('name', cell(1, nB), 'points', cell(1, nB));
    mechanism.joints = jsonData.joints;
    
    for i = 1:nB
        if iscell(jsonData.bodies) 
            bd = jsonData.bodies{i}; 
        else
            bd = jsonData.bodies(i); 
        end
        
        mechanism.bodies(i).name = bd.name;
        p = struct('id', {}, 'location', {});

        for j = 1:numel(bd.points)
            p(j).id = bd.points(j).id;
            p(j).location = bd.points(j).location(:); % Force column vector [X; Y; Z]
        end

        mechanism.bodies(i).points = p;
    end
    
    %% ====================================================================
    % PART 2: GRAPH THEORY (Mapping the Topology)
    % ====================================================================
    % Treat every mechanical body as a "Node" and every joint as an "Edge".
    % We construct a square Adjacency Matrix and fill it with 1s wherever 
    % two bodies share a physical connection. This mathematically maps out 
    % the entire topology of the mechanism.

    adj = zeros(nB, nB);

    for i = 1:numel(mechanism.joints)
        if iscell(mechanism.joints)
            jnt = mechanism.joints{i};
        else
            jnt = mechanism.joints(i);
        end
        
        bNames = jnt.bodies;
        idx1 = find(strcmp({mechanism.bodies.name}, bNames{1}));
        idx2 = find(strcmp({mechanism.bodies.name}, bNames{2}));
        
        % The connection is bidirectional
        adj(idx1, idx2) = 1; 
        adj(idx2, idx1) = 1;
    end
    
    %% ====================================================================
    % PART 3: AUTOMATED LOOP DISCOVERY & VECTOR FORMULATION
    % ====================================================================
    % Feed the adjacency matrix into MATLAB's graph algorithm to find the 
    % the minimum number of independent loops.

    cycles = cyclebasis(graph(adj));
    mechanism.loops = struct();
    
    for k = 1:numel(cycles)
        path = cycles{k}; 
        loopB = [path, path(1)]; % Close the path back to the start node
        vecs = zeros(3, numel(path));
        
        % Initialize arrays to track joint properties along this specific loop
        isPrismatic = false(1, numel(path)); 
        isPrismatic3D = false(1, numel(path));
        axes_list = zeros(3, numel(path));
        jointLabels = cell(1, numel(path));
        jointDirs = zeros(1, numel(path));
        
        % --- Vector Formulation ---
        % Walk along the closed loop. For each body, identify the joint 
        % used to enter the body and the joint used to exit it. Then compute 
        % the 3D vector between these two points.
        
        for i = 1:numel(path)
            % Find the previous body
            if i == 1
                bPrev = mechanism.bodies(loopB(end-1)); 
            else 
                bPrev = mechanism.bodies(loopB(i-1)); 
            end
            
            bCurr = mechanism.bodies(loopB(i));
            bNext = mechanism.bodies(loopB(i+1));
            
            % Find the entry joint (shared with previous body)
            jntEntryIdx = 0;
            for j = 1:numel(mechanism.joints)
                if iscell(mechanism.joints)
                    jnt = mechanism.joints{j}; 
                else
                    jnt = mechanism.joints(j); 
                end

                if all(ismember(jnt.bodies, {bPrev.name, bCurr.name}))
                    jntEntryIdx = j; 
                    break;
                end
            end
            
            if iscell(mechanism.joints)
                entryJnt = mechanism.joints{jntEntryIdx};
            else
                entryJnt = mechanism.joints(jntEntryIdx); 
            end
            lblEntry = entryJnt.label;
            
            % Find the exit joint (shared with next body)
            jntExitIdx = 0;
            for j = 1:numel(mechanism.joints)
                if iscell(mechanism.joints)
                    jnt = mechanism.joints{j}; 
                else 
                    jnt = mechanism.joints(j); 
                end
                
                if all(ismember(jnt.bodies, {bCurr.name, bNext.name}))
                    jntExitIdx = j; 
                    break;
                end
            end

            if iscell(mechanism.joints)
                exitJnt = mechanism.joints{jntExitIdx}; 
            else 
                exitJnt = mechanism.joints(jntExitIdx); 
            end
            lblExit = exitJnt.label;

            % Determine traversal direction (Forward = 1, Backward = -1)
            if strcmp(bCurr.name, exitJnt.bodies{1})
                jointDirs(i) = 1;  
            else
                jointDirs(i) = -1; 
            end

            jointLabels{i} = lblExit;
            
            % Extract the axis of rotation/translation
            if isfield(exitJnt, 'axis')
                axes_list(:, i) = exitJnt.axis(:);
            else
                axes_list(:, i) = [0; 0; 1]; % Default to planar Z-axis if missing
            end

            % --- Kinematic Dimensionality Logic ---
            % 2D: Translates if it touches any rail
            if (isfield(exitJnt, 'type') && strcmpi(exitJnt.type, 'prismatic')) || ...
               (isfield(entryJnt, 'type') && strcmpi(entryJnt.type, 'prismatic'))
                isPrismatic(i) = true;
            end
            
            % 3D: Only slides if the exit joint is a rail
            if isfield(exitJnt, 'type') && strcmpi(exitJnt.type, 'prismatic')
                isPrismatic3D(i) = true;
            end

            % Find the specific [X,Y,Z] coordinates for the entry and exit points
            pEntryIdx = find(strcmp({bCurr.points.id}, lblEntry), 1);
            pExitIdx  = find(strcmp({bCurr.points.id}, lblExit), 1);
            
            if isempty(pEntryIdx) || isempty(pExitIdx)
                error('PARSER ERROR: Body "%s" missing point "%s" or "%s"', bCurr.name, lblEntry, lblExit);
            end
            
            % Calculate the local unrotated geometric vector of the physical link
            vecs(:,i) = bCurr.points(pExitIdx).location - bCurr.points(pEntryIdx).location;
        end
        
        % Store the loop data
        mechanism.loops(k).bodyIndices = path;
        mechanism.loops(k).vectors = vecs;
        mechanism.loops(k).axes = axes_list;
        mechanism.loops(k).jointLabels = jointLabels;
        mechanism.loops(k).isPrismatic = isPrismatic;
        mechanism.loops(k).isPrismatic3D = isPrismatic3D;
        mechanism.loops(k).dirs = jointDirs;
    end
end