% =========================================================================
% linkageVisualizer.m
% =========================================================================
% Author:        Benjamin B Bolduc
% Last Updated:  March 2026
%
% Description: 
%   The function takes the JSON mechanism and the final solved joint 
%   parameters, performs forward kinematics to find the global 3D 
%   coordinates, and plots the mechanism.
% =========================================================================

function linkageVisualizer(figHandle, mechanism, thetaSolved, isDynamic, showLabels)
    if nargin < 4, isDynamic = false; end
    if nargin < 5, showLabels = true; end % Default to showing labels if not specified
    
    %% --- SETUP AND INITIALIZATION ---
    if isempty(figHandle)
        figHandle = figure('Name', 'Mechanism Visualization'); 
    end

    set(figHandle, 'Color', 'w'); 
    ax = gca;
    [az, el] = view(ax);
    cla(ax); 
    hold(ax, 'on'); 
    grid(ax, 'on');
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
    
    nBodies = numel(mechanism.bodies);
    
    axesCell = num2cell(mechanism.loops(1).axes, 1);
    vectorsCell = num2cell(mechanism.loops(1).vectors, 1);
    isPlanar = isMechanismPlanar(axesCell, vectorsCell, 1e-6);
    is3D = ~isPlanar;
    
    %% --- THE FORWARD KINEMATICS SOLVER ---
    [bodyPos, bodyRot, placed] = solveGlobalPoses(mechanism, thetaSolved, isPlanar);
    
    %% --- 1. CALCULATE GLOBAL JOINT POSITIONS --- 
    jointWorldPos = containers.Map();

    for j = 1:numel(mechanism.joints)
        if iscell(mechanism.joints)
            jnt = mechanism.joints{j}; 
        else 
            jnt = mechanism.joints(j); 
        end
        
        for bName = jnt.bodies
            bIdx = find(strcmp({mechanism.bodies.name}, bName{1}));
            if placed(bIdx)
                loc_local = getLoc(mechanism.bodies(bIdx), jnt.label);
                jointWorldPos(jnt.label) = bodyPos{bIdx} + bodyRot{bIdx} * loc_local(:);
                break;
            end
        end
    end
    
    %% --- 2. DRAW BODIES ---
    colors = lines(nBodies);

    for i = 1:nBodies
        if ~placed(i) 
            continue; 
        end
        
        body = mechanism.bodies(i);
        pts_draw = [];
        
        nDraw = numel(body.points);
        if (strcmp(body.name, 'Frame') || strcmp(body.name, 'Platform')) && nDraw == 7
            nDraw = 6; 
        end
        
        for k = 1:nDraw
            pt_vec = body.points(k).location(:); 
            pts_draw(:, end+1) = bodyPos{i} + bodyRot{i} * pt_vec;
        end
        
        if size(pts_draw, 2) > 2
            pts_draw = [pts_draw, pts_draw(:,1)]; 
        end
        
        plot3(ax, pts_draw(1,:), pts_draw(2,:), pts_draw(3,:), '-', 'Color', colors(i,:), 'LineWidth', 5);
    end
    
    %% --- 3. PROCESS JOINT MOTION TRAILS AND SPATIAL GROUPING ---
    trails = getappdata(figHandle, 'jointTrails');
    if isempty(trails)
        trails = containers.Map(); 
    end
    
    jLabels = keys(jointWorldPos);
    jCoords = values(jointWorldPos);
    uniqueCoords = []; 
    uniqueNames = {};
    
    for i = 1:numel(jLabels)
        pos = jCoords{i};
        lbl = jLabels{i};
        
        if isKey(trails, lbl)
            trails(lbl) = [trails(lbl), pos];
        else
            trails(lbl) = pos;
        end

        tr = trails(lbl);

        if size(tr, 2) > 1
            plot3(ax, tr(1,:), tr(2,:), tr(3,:), '.', 'Color', [0.4 0.4 0.4], 'MarkerSize', 4);
        end
        
        found = false;

        for k = 1:size(uniqueCoords, 1)
            if norm(pos - uniqueCoords(k,:)') < 1e-4
                uniqueNames{k} = [uniqueNames{k}, ', ', jLabels{i}];
                found = true; 
                break;
            end
        end

        if ~found
            uniqueCoords(end+1, :) = pos';
            uniqueNames{end+1} = jLabels{i};
        end
    end

    setappdata(figHandle, 'jointTrails', trails); 
    
    %% --- 4. PARAMETRIC JOINT RENDERING ---
    for i = 1:size(uniqueCoords, 1)
        pos = uniqueCoords(i, :);
        lblName = uniqueNames{i};

        % 1. Split the label to look at the exact joints stacked here
        splitNames = strtrim(strsplit(lblName, ','));
        firstJntName = splitNames{1};
        numJointsAtNode = numel(splitNames);

        % 2. Look up the joint type from JSON data
        jType = 'revolute'; % Default assumption
        for jIdx = 1:numel(mechanism.joints)
            if iscell(mechanism.joints), tempJnt = mechanism.joints{jIdx}; else, tempJnt = mechanism.joints(jIdx); end
            if strcmp(tempJnt.label, firstJntName)
                if isfield(tempJnt, 'type')
                    jType = lower(tempJnt.type);
                end
                break;
            end
        end

        % 3. Draw the correct shape based on pure physics
        if strcmp(jType, 'prismatic')
            mType = 's'; mSize = 10; mColor = [0.2 0.8 0.8]; % Prismatic (Square)
        elseif numJointsAtNode >= 3
            mType = 'p'; mSize = 12; mColor = [0.93 0.69 0.13]; % Spherical (Star)
        elseif numJointsAtNode == 2
            mType = 'd'; mSize = 8; mColor = [0.85 0.33 0.1]; % Universal (Diamond)
        else
            mType = 'o'; mSize = 6; mColor = [0.64 0.08 0.18]; % Revolute (Circle)
        end

        plot3(ax, pos(1), pos(2), pos(3), mType, 'MarkerSize', mSize, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', mColor);
        
        % Only draw the label if the checkbox is checked
        if showLabels
            text(ax, pos(1) + 0.02, pos(2) + 0.02, pos(3) + 0.02, lblName, 'Color', 'k', 'FontSize', 10, 'FontWeight', 'bold');
        end
    end
    
%% --- 5. JOINT LEGEND OVERLAY ---
    h_rev = plot3(ax, NaN, NaN, NaN, 'o', 'MarkerSize', 6, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.64 0.08 0.18]);
    h_pri = plot3(ax, NaN, NaN, NaN, 's', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.2 0.8 0.8]);
    h_uni = plot3(ax, NaN, NaN, NaN, 'd', 'MarkerSize', 8, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.85 0.33 0.1]);
    h_sph = plot3(ax, NaN, NaN, NaN, 'p', 'MarkerSize', 12, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.93 0.69 0.13]);

    legend(ax, [h_rev, h_pri, h_uni, h_sph], {'Revolute', 'Prismatic', 'Universal', 'Spherical'}, ...
        'Location', 'northeast', 'TextColor', 'k', 'Color', 'w', 'EdgeColor', 'k', 'AutoUpdate', 'off');

    % Camera logic optimization
    if is3D
        if az == 0 && el == 90
            view(3);
        else
            view(ax, az, el);
        end
        rotate3d on;
    else
        view(2);
    end
end

function [bodyPos, bodyRot, placed] = solveGlobalPoses(mechanism, thetaSolved, isPlanar)
    nB = numel(mechanism.bodies);
    bodyPos = cell(nB, 1); 
    bodyRot = cell(nB, 1); 
    placed = false(nB, 1);
    
    if isPlanar && isfield(thetaSolved, 'GlobalAngles')
        for i = 1:nB
            phi = thetaSolved.GlobalAngles(i);
            bodyRot{i} = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
        end
    else
        bodyRot{1} = eye(3); 
    end

    bodyPos{1} = [0;0;0]; placed(1) = true;
    
    for pass = 1:15 
        for j = 1:numel(mechanism.joints) 
            if iscell(mechanism.joints)
                jnt = mechanism.joints{j}; 
            else 
                jnt = mechanism.joints(j); 
            end

            idA = find(strcmp({mechanism.bodies.name}, jnt.bodies{1}));
            idB = find(strcmp({mechanism.bodies.name}, jnt.bodies{2}));
            val = 0; 
            
            if isfield(thetaSolved, jnt.label)
                val = thetaSolved.(jnt.label); 
            end
            
            ax_vec = [0;0;1]; 
            
            if isfield(jnt, 'axis')
                ax_vec = jnt.axis(:); 
            end
            
            isPrism = isfield(jnt, 'type') && strcmpi(jnt.type, 'prismatic');
            
            if placed(idA) && ~placed(idB)
                if ~isPlanar
                    if isPrism
                        bodyRot{idB} = bodyRot{idA}; 
                    else
                        bodyRot{idB} = bodyRot{idA} * axang2rotm([ax_vec(:)' val]); 
                    end
                end
                
                offset = [0;0;0]; 
                
                if isPrism
                    offset = bodyRot{idA} * ax_vec * val; 
                end
                
                P_j_A = bodyPos{idA} + bodyRot{idA} * getLoc(mechanism.bodies(idA), jnt.label);
                bodyPos{idB} = (P_j_A + offset) - bodyRot{idB} * getLoc(mechanism.bodies(idB), jnt.label);
                placed(idB) = true;
                
            elseif placed(idB) && ~placed(idA)
                if ~isPlanar
                    if isPrism
                        bodyRot{idA} = bodyRot{idB}; 
                    else 
                        bodyRot{idA} = bodyRot{idB} * axang2rotm([ax_vec(:)' val])'; 
                    end
                end

                offset = [0;0;0]; 
                
                if isPrism
                    offset = bodyRot{idA} * ax_vec * val; 
                end

                P_j_B = bodyPos{idB} + bodyRot{idB} * getLoc(mechanism.bodies(idB), jnt.label);
                bodyPos{idA} = (P_j_B - offset) - bodyRot{idA} * getLoc(mechanism.bodies(idA), jnt.label);
                placed(idA) = true;
            end
        end
    end
end

function loc = getLoc(body, id)
    loc = [0; 0; 0]; 
    for k = 1:numel(body.points)
        if strcmp(body.points(k).id, id)
            loc = body.points(k).location(:); 
            return; 
        end
    end
end