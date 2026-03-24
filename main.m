% =========================================================================
% main.m
% =========================================================================
% Author:        Benjamin B Bolduc
% Last updated:  March 2026
%
% Description: 
%   Front-end GUI to load a mechanism JSON, define initial guesses, 
%   select input joints, and automatically launch the solver.
% =========================================================================

function main()
    clc;

    bgDark = [0.15 0.15 0.15];
    bgEdit = [0.25 0.25 0.25];
    fgText = [1 1 1];
    
    % =====================================================================
    % 1. File Explorer Prompt
    % =====================================================================
    [fileName, filePath] = uigetfile('*.json', 'Select Mechanism JSON', 'linkages/');
    if isequal(fileName,0)
        disp('User canceled file selection.');
        return;
    end
    jsonFile = fullfile(filePath, fileName);
    
    % =====================================================================
    % 2. Preliminary JSON Parse & Filter
    % =====================================================================
    jsonData = jsondecode(fileread(jsonFile));
    
    if iscell(jsonData.joints)
        joints = jsonData.joints;
    else
        joints = num2cell(jsonData.joints); 
    end
    
    % Extract only unique joint labels
    allLabels = cell(1, numel(joints));
    for i = 1:numel(joints)
        allLabels{i} = joints{i}.label;
    end
    uniqueLabels = unique(allLabels);
    nUnique = numel(uniqueLabels);
    
    % =====================================================================
    % 3. Build the UI Window
    % =====================================================================
    f = uifigure('Name', ['Setup: ' fileName], 'Position', [200 200 480 600], 'Color', bgDark);
               
    pnl = uipanel(f, 'Position', [0 100 480 500], 'Scrollable', 'on', 'BackgroundColor', bgDark, 'BorderType', 'none');
    
    contentHeight = max(500, 50 + (nUnique * 30));
    uilabel(pnl, 'Position', [20, contentHeight-30, 100, 20], 'Text', 'Joint Label', ...
            'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'FontColor', fgText);
    uilabel(pnl, 'Position', [150, contentHeight-30, 80, 20], 'Text', 'Is Input?', ...
            'FontWeight', 'bold', 'FontColor', fgText);
    uilabel(pnl, 'Position', [250, contentHeight-30, 120, 20], 'Text', 'Guess / Target', ...
            'FontWeight', 'bold', 'FontColor', fgText);
            
    chkInputs = gobjects(nUnique, 1);
    edtGuesses = gobjects(nUnique, 1);
    yOffset = contentHeight - 60;
    
    for i = 1:nUnique
        uilabel(pnl, 'Position', [20, yOffset, 120, 20], 'Text', uniqueLabels{i}, ...
                'HorizontalAlignment', 'left', 'FontColor', fgText);
        
        chkInputs(i) = uicheckbox(pnl, 'Position', [180, yOffset, 20, 20], 'Text', '');
        
        % Use 'text' instead of 'numeric' to allow arrays like "100, -100"
        % as inputs
        edtGuesses(i) = uieditfield(pnl, 'text', 'Position', [260, yOffset, 100, 25], 'Value', '0', ...
                                    'BackgroundColor', bgEdit, 'FontColor', fgText);
        
        yOffset = yOffset - 30;
    end
    
    uilabel(f, 'Position', [20, 50, 100, 20], 'Text', 'Slider Min Limit:', ...
            'HorizontalAlignment', 'left', 'FontColor', fgText);
    edtMin = uieditfield(f, 'numeric', 'Position', [120, 50, 60, 25], 'Value', 0, ...
                         'BackgroundColor', bgEdit, 'FontColor', fgText);
                         
    uilabel(f, 'Position', [200, 50, 100, 20], 'Text', 'Slider Max Limit:', ...
            'HorizontalAlignment', 'left', 'FontColor', fgText);
    edtMax = uieditfield(f, 'numeric', 'Position', [300, 50, 60, 25], 'Value', 360, ...
                         'BackgroundColor', bgEdit, 'FontColor', fgText);
                         
    uibutton(f, 'Position', [165, 10, 150, 30], 'Text', 'LAUNCH SOLVER', ...
             'FontWeight', 'bold', 'BackgroundColor', [0.2 0.6 0.2], 'FontColor', 'w', ...
             'ButtonPushedFcn', @launchCallback);
             
    chkLabels = uicheckbox(f, 'Position', [330, 15, 100, 20], 'Text', 'Show Labels', ...
                           'FontColor', fgText, 'Value', 1);

    chkSlider = uicheckbox(f, 'Position', [20, 15, 140, 20], 'Text', 'Launch Dynamic Slider', ...
                           'FontColor', fgText, 'Value', 1); % Defaults to checked
                           
    % =====================================================================
    % 4. Launch Callback
    % =====================================================================
    function launchCallback(~, ~)
        thetaGuesses = struct();
        inputJoints = {};
        thetaInputs = struct(); 
        
        for k = 1:nUnique
            val = str2num(edtGuesses(k).Value); 
            if isempty(val), val = 0; end
            
            isPrism = false;
            for j = 1:numel(joints)
                if strcmp(joints{j}.label, uniqueLabels{k})
                    if isfield(joints{j}, 'type') && strcmpi(joints{j}.type, 'prismatic')
                        isPrism = true;
                    end
                    break;
                end
            end
            
            if isPrism
                convertedVal = val; 
            else
                convertedVal = deg2rad(val); 
            end
            
            thetaGuesses.(uniqueLabels{k}) = convertedVal; 
            if chkInputs(k).Value == 1 
                inputJoints{end+1} = uniqueLabels{k};
                thetaInputs.(uniqueLabels{k}) = convertedVal(1); 
            end
        end
        
        minLim = edtMin.Value;
        maxLim = edtMax.Value;
        if isnan(minLim), minLim = 0; end
        if isnan(maxLim), maxLim = 360; end
        angle_limits = repmat([minLim, maxLim], numel(inputJoints), 1);
        
        showLabelsFlag = chkLabels.Value;
        launchSliderFlag = chkSlider.Value; % Capture the slider checkbox state
        
        close(f);
        
        mechanism = solvePosKin(jsonFile);
        
        fprintf('\n======================================================\n');
        fprintf(' STATIC POSTURE VERIFICATION: %s\n', jsonFile);
        fprintf('======================================================\n');
        [thetaSolved, maxResidual, posError, angError, exitflag] = closureEqsSolver(thetaInputs, thetaGuesses, mechanism);
        
        if exitflag > 0
            fprintf('SUCCESS: Mechanism loops closed perfectly.\n');
            fprintf(' -> Max Translation Error:     %.4e m\n', posError);
            fprintf(' -> Max Angular Error: %.4e deg\n\n', angError);
            
            % --- RESTORED CONSOLE REPORTING BLOCK ---
            fprintf('--- Solved Joint Angles & Extensions ---\n');
            fields = fieldnames(thetaSolved);
            for i = 1:numel(fields)
                jointName = fields{i};
                if ~strcmp(jointName, 'GlobalAngles')
                    b1 = 'Unknown'; b2 = 'Unknown';
                    isPrism = false;
                    for j = 1:numel(mechanism.joints)
                        if iscell(mechanism.joints), jnt = mechanism.joints{j}; else, jnt = mechanism.joints(j); end
                        if strcmp(jnt.label, jointName)
                            b1 = jnt.bodies{1}; b2 = jnt.bodies{2};
                            if isfield(jnt, 'type') && strcmpi(jnt.type, 'prismatic')
                                isPrism = true;
                            end
                            break;
                        end
                    end
                    
                    if isPrism
                        val = thetaSolved.(jointName); 
                        fprintf(' %s (%s -> %s): %.3f m\n', jointName, b1, b2, val);
                    else
                        val = rad2deg(thetaSolved.(jointName)); 
                        fprintf(' %s (%s -> %s): %.2f deg\n', jointName, b1, b2, val);
                    end
                end
            end
            fprintf('\n');
            % ----------------------------------------
            
            staticFig = figure('Name', ['Verification Report: ' jsonFile], ...
                               'Position', [150, 150, 800, 600], 'Color', bgDark);
            
            linkageVisualizer(staticFig, mechanism, thetaSolved, false, showLabelsFlag);
            
            ax = gca;
            set(ax, 'Position', [0.35 0.1 0.60 0.8]);
            axis(ax, 'equal');
            x_lim = ax.XLim; y_lim = ax.YLim;
            padX = (x_lim(2) - x_lim(1)) * 0.2; padY = (y_lim(2) - y_lim(1)) * 0.2; 
            set(ax, 'XLim', [x_lim(1)-padX, x_lim(2)+padX], 'YLim', [y_lim(1)-padY, y_lim(2)+padY]);
            grid(ax, 'on');
            
            titleStr = sprintf('Static Posture Verification: %s\nMax Translation Error: %.2e m | Max Angular Error: %.2e°', ...
                               jsonFile, posError, angError);
            title(ax, titleStr, 'Interpreter', 'none', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
            
            startingGuess = thetaSolved;
        else
            fprintf('FAILURE: Impossible configuration or singularity reached.\n');
            startingGuess = thetaGuesses;
        end
        
        if launchSliderFlag
            launchLinkageSlider(mechanism, inputJoints, startingGuess, angle_limits, showLabelsFlag);
        else
            fprintf('Dynamic slider launch bypassed.\n');
        end
    end
end