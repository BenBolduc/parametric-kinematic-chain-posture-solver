% =========================================================================
% launchLinkageSlider.m
% =========================================================================
% Author:        Benjamin B Bolduc
% Last Updated:  March 2026
%
% Description: 
%   This generates a UI window with sliders that allows the user to 
%   manually sweep the input joints through a range of angles. 
% =========================================================================

function launchLinkageSlider(mechanism, inputJoints, thetaGuesses, angle_limits, showLabelsFlag)
    if nargin < 5, showLabelsFlag = true; end % Default failsafe
    
    currentGuess = thetaGuesses;
    n_inputs = numel(inputJoints);

    isInputPrism = false(n_inputs, 1);
    for i = 1:n_inputs
        for j = 1:numel(mechanism.joints)
            if iscell(mechanism.joints), jnt = mechanism.joints{j}; else, jnt = mechanism.joints(j); end
            if strcmp(jnt.label, inputJoints{i})
                if isfield(jnt, 'type') && strcmpi(jnt.type, 'prismatic')
                    isInputPrism(i) = true;
                end
                break;
            end
        end
    end
    
    % Change figure color to white
    fig = figure('Name', 'Linkage Dynamic Explorer', ...
                 'Color', 'w', ...
                 'Position', [100 100 1000 800], ...
                 'NumberTitle', 'off');

    setappdata(fig, 'jointTrails', containers.Map());
             
    rowHeight = 0.08;
    bottomSpaceNeeded = n_inputs * rowHeight; 
    axBottom = bottomSpaceNeeded + 0.05; 
    axHeight = max(0.4, 0.95 - axBottom); 
    
    ax = axes('Parent', fig, 'Position', [0.1 axBottom 0.8 axHeight]);
    
    sliders = gobjects(n_inputs, 1);
    labels = gobjects(n_inputs, 1);

    lockedX = []; 
    lockedY = []; 
    lockedZ = [];

    inputHistory_S_deg = NaN(2, n_inputs); 
    unknownHistory_Struct = {struct(), struct()}; 
    
    for i = 1:n_inputs
        minAng = angle_limits(i, 1);
        maxAng = angle_limits(i, 2);
        
        startVal = minAng;
        if isfield(thetaGuesses, inputJoints{i})
            if isInputPrism(i)
                % Keep prismatic inputs as meters
                startVal = thetaGuesses.(inputJoints{i});
            else
                % Convert revolute inputs from radians to degrees for the slider
                startVal = rad2deg(thetaGuesses.(inputJoints{i}));
            end
        end
        
        y_pos = bottomSpaceNeeded - (i-1)*rowHeight - 0.05; 
        
        labels(i) = uicontrol('Parent', fig, 'Style', 'text', ...
            'Units', 'normalized', 'Position', [0.1 y_pos+0.035 0.8 0.03], ...
            'String', sprintf('%s: %.1f deg', inputJoints{i}, startVal), ...
            'BackgroundColor', 'w', 'ForegroundColor', 'k', ...
            'FontSize', 12, 'FontWeight', 'bold');
            
        sliders(i) = uicontrol('Parent', fig, 'Style', 'slider', ...
            'Units', 'normalized', 'Position', [0.2 y_pos 0.6 0.035], ...
            'Min', minAng, 'Max', maxAng, 'Value', startVal);
            
        addlistener(sliders(i), 'Value', 'PostSet', @(src, event) updateUI());
    end
    updateUI();
    
    function updateUI()
        thetaInputs = struct();
        title_str = '';
        
        for k = 1:n_inputs
            val = sliders(k).Value;

            if isInputPrism(k)
                thetaInputs.(inputJoints{k}) = val; 
                unitStr = 'm';
            else
                thetaInputs.(inputJoints{k}) = deg2rad(val); 
                unitStr = 'deg';
            end

            labels(k).String = sprintf('%s: %.2f %s', inputJoints{k}, val, unitStr);
            title_str = [title_str, sprintf('%s: %.2f %s | ', inputJoints{k}, val, unitStr)];
        end
        
        [thetaSolved, ~, posError_mm, ~, exitflag] = closureEqsSolver(thetaInputs, currentGuess, mechanism);
        
        if exitflag > 0
            linkageVisualizer(fig, mechanism, thetaSolved, true, showLabelsFlag);
            currentGuess = thetaSolved; 
            
            if isempty(lockedX)
                axis(ax, 'auto');
                axis(ax, 'equal');
                drawnow;

                lockedX = ax.XLim + [-0.25, 0.25];
                lockedY = ax.YLim + [-0.25, 0.5];
                lockedZ = ax.ZLim + [-0.25, 0.25];
            else
                % Force the axes to stay frozen
                ax.XLim = lockedX;
                ax.YLim = lockedY;
                ax.ZLim = lockedZ;
            end

            title(ax, sprintf('%sMax Gap: %.2e mm', title_str, posError_mm), 'Color', 'k', 'FontSize', 14);
        else
            title(ax, 'SINGULARITY/LIMIT REACHED', 'Color', 'r', 'FontSize', 14);
        end
        drawnow limitrate; 
    end
end