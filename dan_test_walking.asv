% Prototype walking script for humanoid robot

clc
close all

% addpath(strcat(pwd(),'/preview_control'))
% addpath(strcat(pwd(),'/graphing'))


% needs the file regen.m
if ~exist('REGENERATE_ROBOT_DATA','var')
    REGENERATE_ROBOT_DATA = 1
end

if exist('robot_config.mat','file') ~= 2 || REGENERATE_ROBOT_DATA == true
    disp 'regenerating robot preview controller'
    REGENERATE_ROBOT_DATA = false;
    SetupBipedRobot3
    walk_config;

    % defining the zmp system. Same for x and y
    T = 0.005;
    previewLength = 1.6/T;

    zc = uLINK(BODY).p(3); % height of the CoM above the ground
    gconst = 9.8; % m/s 

    A = [1 T T^2;
         0 1 T;
         0 0 1];

    B = [T^3/6; T^2/2; T];

    C = [ 1 0 -zc/gconst];

    D = 0;

    Qe = 1;
    Qx = diag([0 0 0]);
    R = 1e-6;

    % calculate the preview control gains and the simulation system
    sys = ss(A,B,C,D,T);
    [sys_t,Gi,Gx,Gd]= preview_control(sys,previewLength,Qe,Qx,R);

    save('robot_config')
else
    disp 'got here'
    load('robot_config.mat')
end

% Constants
tsPerStep = round(previewLength/1.5);
numStepGroups = 2;
xStepSize = .2;
yStepSize = .1;
stepIdx = 1;
xErr = 0;
yErr = 0;
zSwingHeight = 0.15;
RIGHT_FOOT = 0;
LEFT_FOOT = 1;
stanceFoot = LEFT_FOOT;
stepGroupSize = 2;
tsPerFrame = 40;
% linkLength = .35; % defined in SetupBipedRobot3

xq0 = [0 uLINK(BODY).p(1), 0 0]';
yq0 = [0 uLINK(BODY).p(2), 0 0]';

% always set the number of steps be an even number
steps = [ 0 0 0     0    ;
          0 0 -0.1  0.1 ];
t = 0:T:100000;

robotViewH1 = figure('units','normalized','position',[0 0.65 0.2 0.2]);
robotViewH2 = figure('units','normalized','position',[0 0.35 0.2 0.2]);
robotViewH3 = figure('units','normalized','position',[0 0.05 0.2 0.2]);
graphPlotH = figure('units','normalized','position',[.225 .5 0.3 0.35]);

% need an initialization phase where you just move the COM and keep both
% feet on the ground.

%%%%%%%%%%%%
% Starting %
%%%%%%%%%%%%

xZmpRef = footsteps(steps(1,:), tsPerStep);
yZmpRef = footsteps(steps(2,:), tsPerStep);

[xq, xZmp, ux] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
    xZmpRef, xq0(2:4), xErr, previewLength);
[yq, yZmp, uy] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
    yZmpRef, yq0(2:4), yErr, previewLength);

xComRef = xq(2,:);
yComRef = yq(2,:);
stanceFoot

figure(graphPlotH)
subplot(2,1,1)
cla
hold on
plot(xComRef)
plot(xZmpRef)
plot(xZmp)
legend('xComRef','xZmpRef','xZmp')
subplot(2,1,2)
cla
hold on
plot(yComRef)
plot(yZmpRef)
plot(yZmp)
legend('yComRef','yZmpRef','yZmp')

for ii = 1:2 % each step
    
    switch(stanceFoot)
        case(RIGHT_FOOT)
            stanceFootLink = RLEG_J5;
            stanceHipLink = RLEG_J0;
            swingHipLink = LLEG_J0;
            swingFootLink = LLEG_J5;
        case(LEFT_FOOT)
            stanceFootLink = LLEG_J5;
            stanceHipLink = LLEG_J0;
            swingHipLink = RLEG_J0;
            swingFootLink = RLEG_J5;
        otherwise
            disp 'stance foot isn''t being used correctly'
    end
    
    footPlacementPair = [[uLINK(swingFootLink).p(1); ...
        uLINK(swingFootLink).p(2)] steps(:,1)];
    
%     [xSwing, ySwing, zSwing] = ...
%         cubic_spline_trajectory( footPlacementPair(1,:), ...
%         footPlacementPair(2,:), ...
%         [GND, zSwingHeight], tsPerStep);
    
    for i=1:tsPerFrame:tsPerStep
        uLINK(BODY).p = [xComRef((ii-1)*tsPerStep+i), ...
            yComRef((ii-1)*tsPerStep+i), uLINK(BODY).p(3)]';
        uLINK(BODY).R = eye(3);
        
        % set stance foot
        stancefoot.p = [uLINK(stanceFootLink).p(1), ...
            uLINK(stanceFootLink).p(2), GND]';
        stancefoot.R = RPY2R([0, 0, 0]);  %  -pi/4 < q < pi/4
        qRStance = IK_leg(uLINK(BODY), (-1)^stanceFoot*-0.1, linkLength, ...
            linkLength, stancefoot);
        
        % set swing foot
        swingfoot.p = [uLINK(swingFootLink).p(1), uLINK(swingFootLink).p(2), GND]';
        swingfoot.R = RPY2R([0, 0, 0]);
        qRSwing = IK_leg(uLINK(BODY), (-1)^stanceFoot*0.1, linkLength, ...
            linkLength, swingfoot);
        
        for j=0:5
            uLINK(stanceHipLink+j).q = qRStance(j+1);
            uLINK(swingHipLink+j).q = qRSwing(j+1);
        end
        ForwardKinematics(1);
        
        % graphing
        figure(robotViewH1)
        clf
        DrawRobot
        view([0,0])
        
        figure(robotViewH2)
        clf
        DrawRobot
        view([90,0])
        
        figure(robotViewH3)
        clf
        DrawRobot
        view([30,25])
        
        figure(graphPlotH)
        subplot(2,1,1)
        hold on
        plot((ii-1)*tsPerStep+i,xComRef((ii-1)*tsPerStep+i),'o')
        plot((ii-1)*tsPerStep+i,xZmp((ii-1)*tsPerStep+i),'x')
        legend('xComRef','xZmpRef','xZmp')
        subplot(2,1,2)
        hold on
        plot((ii-1)*tsPerStep+i,yComRef((ii-1)*tsPerStep+i),'o')
        plot((ii-1)*tsPerStep+i,yZmp((ii-1)*tsPerStep+i),'x')
        legend('yComRef','yZmpRef','yZmp')
        drawnow
        pause(0.001)
    end
    
    steps = [steps(:,2:end), [steps(1,end) + xStepSize; ...
        (-1)^stanceFoot*yStepSize]]
    
    stanceFoot = mod(stanceFoot + 1, 2);
end

% reinitialize for next steps
xErr = xq(1,tsPerStep*stepGroupSize+1 );
yErr = yq(1,tsPerStep*stepGroupSize+1);
xq0 = xq(:,tsPerStep*stepGroupSize+1);
yq0 = yq(:,tsPerStep*stepGroupSize+1);


%%%%%%%%%%%
% Walking %
%%%%%%%%%%%
for stepGroupIdx = 1:numStepGroups % group of steps
    stepGroupIdx
    xZmpRef = footsteps(steps(1,:), tsPerStep);
    yZmpRef = footsteps(steps(2,:), tsPerStep);
    
    [xq, xZmp, ux] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
        xZmpRef, xq0(2:4), xErr, previewLength);
    [yq, yZmp, uy] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
        yZmpRef, yq0(2:4), yErr, previewLength);
    
    xComRef = xq(2,:);
    yComRef = yq(2,:);
    stanceFoot

    figure(graphPlotH)
    subplot(2,1,1)
    cla
    hold on
    plot(xComRef)
    plot(xZmpRef)
    plot(xZmp)
    legend('xComRef','xZmpRef','xZmp')
    subplot(2,1,2)
    cla
    hold on
    plot(yComRef)
    plot(yZmpRef)
    plot(yZmp)
    legend('yComRef','yZmpRef','yZmp')
    
    for ii = 1:stepGroupSize % each step

        switch(stanceFoot)
            case(RIGHT_FOOT)
                stanceFootLink = RLEG_J5;
                stanceHipLink = RLEG_J0;
                swingHipLink = LLEG_J0;
                swingFootLink = LLEG_J5;
            case(LEFT_FOOT)
                stanceFootLink = LLEG_J5;
                stanceHipLink = LLEG_J0;
                swingHipLink = RLEG_J0;
                swingFootLink = RLEG_J5;
            otherwise
                disp 'stance foot isn''t being used correctly'
        end

        footPlacementPair = [[uLINK(swingFootLink).p(1); ...
                              uLINK(swingFootLink).p(2)] steps(:,1)];
        
        [xSwing, ySwing, zSwing] = ...
            cubic_spline_trajectory( footPlacementPair(1,:), ...
                                     footPlacementPair(2,:), ...
                                     [GND, zSwingHeight], tsPerStep);
                                 
        for i=1:tsPerFrame:tsPerStep
            uLINK(BODY).p = [xComRef((ii-1)*tsPerStep+i), ...
                          yComRef((ii-1)*tsPerStep+i), uLINK(BODY).p(3)]';
            uLINK(BODY).R = eye(3);

            % set stance foot
            stancefoot.p = [uLINK(stanceFootLink).p(1), ...
                            uLINK(stanceFootLink).p(2), GND]';
            stancefoot.R = RPY2R([0, 0, 0]);  %  -pi/4 < q < pi/4
            qRStance = IK_leg(uLINK(BODY), (-1)^stanceFoot*-0.1, linkLength, ...
                linkLength, stancefoot);

            % set swing foot
            swingfoot.p = [xSwing(i), ySwing(i), zSwing(i)]';
            swingfoot.R = RPY2R([0, 0, 0]);
            qRSwing = IK_leg(uLINK(BODY), (-1)^stanceFoot*0.1, linkLength, ...
                linkLength, swingfoot);

            for j=0:5
                uLINK(stanceHipLink+j).q = qRStance(j+1);
                uLINK(swingHipLink+j).q = qRSwing(j+1);
            end
            ForwardKinematics(1);
            
            % graphing
            figure(robotViewH1)
            clf
            DrawRobot
            view([0,0])
            
            figure(robotViewH2)
            clf
            DrawRobot
            view([90,0])
            
            figure(robotViewH3)
            clf
            DrawRobot
            view([30,25])
            
            figure(graphPlotH)
            subplot(2,1,1)
            hold on
            plot((ii-1)*tsPerStep+i,xComRef((ii-1)*tsPerStep+i),'o')
            plot((ii-1)*tsPerStep+i,xZmp((ii-1)*tsPerStep+i),'x')
            legend('xComRef','xZmpRef','xZmp')
            subplot(2,1,2)
            hold on
            plot((ii-1)*tsPerStep+i,yComRef((ii-1)*tsPerStep+i),'o')
            plot((ii-1)*tsPerStep+i,yZmp((ii-1)*tsPerStep+i),'x')
            legend('yComRef','yZmpRef','yZmp')
            drawnow
            pause(0.001)
        end

        steps = [steps(:,2:end), [steps(1,end) + xStepSize; ...
                 (-1)^stanceFoot*yStepSize]]

        stanceFoot = mod(stanceFoot + 1, 2);
    end
    
    % reinitialize for next steps
    xErr = xq(1,tsPerStep*stepGroupSize+1 );
    yErr = yq(1,tsPerStep*stepGroupSize+1);
    xq0 = xq(:,tsPerStep*stepGroupSize+1);
    yq0 = yq(:,tsPerStep*stepGroupSize+1);
end

%%%%%%%%%%%%%%%%%%%%
% Coming to a stop %
%%%%%%%%%%%%%%%%%%%%

finalx = steps(1,2)
steps = [ steps(:,1:2) [finalx finalx; -0.1 0.1] ]; 

xZmpRef = footsteps(steps(1,:), tsPerStep);
yZmpRef = footsteps(steps(2,:), tsPerStep);

[xq, xZmp, ux] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
    xZmpRef, xq0(2:4), xErr, previewLength);
[yq, yZmp, uy] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
    yZmpRef, yq0(2:4), yErr, previewLength);

xComRef = xq(2,:);
yComRef = yq(2,:);
stanceFoot

figure(graphPlotH)
subplot(2,1,1)
cla
hold on
plot(xComRef)
plot(xZmpRef)
plot(xZmp)
legend('xComRef','xZmpRef','xZmp')
subplot(2,1,2)
cla
hold on
plot(yComRef)
plot(yZmpRef)
plot(yZmp)
legend('yComRef','yZmpRef','yZmp')

for ii = 1:2 % each step

    switch(stanceFoot)
        case(RIGHT_FOOT)
            stanceFootLink = RLEG_J5;
            stanceHipLink = RLEG_J0;
            swingHipLink = LLEG_J0;
            swingFootLink = LLEG_J5;
        case(LEFT_FOOT)
            stanceFootLink = LLEG_J5;
            stanceHipLink = LLEG_J0;
            swingHipLink = RLEG_J0;
            swingFootLink = RLEG_J5;
        otherwise
            disp 'stance foot isn''t being used correctly'
    end

    footPlacementPair = [[uLINK(swingFootLink).p(1); ...
                          uLINK(swingFootLink).p(2)] steps(:,1)];

    [xSwing, ySwing, zSwing] = ...
        cubic_spline_trajectory( footPlacementPair(1,:), ...
                                 footPlacementPair(2,:), ...
                                 [GND, zSwingHeight], tsPerStep);

    for i=1:tsPerFrame:tsPerStep
        uLINK(BODY).p = [xComRef((ii-1)*tsPerStep+i), ...
                      yComRef((ii-1)*tsPerStep+i), uLINK(BODY).p(3)]';
        uLINK(BODY).R = eye(3);

        % set stance foot
        stancefoot.p = [uLINK(stanceFootLink).p(1), ...
                        uLINK(stanceFootLink).p(2), GND]';
        stancefoot.R = RPY2R([0, 0, 0]);  %  -pi/4 < q < pi/4
        qRStance = IK_leg(uLINK(BODY), (-1)^stanceFoot*-0.1, linkLength, ...
            linkLength, stancefoot);

        % set swing foot
        swingfoot.p = [xSwing(i), ySwing(i), zSwing(i)]';
        swingfoot.R = RPY2R([0, 0, 0]);
        qRSwing = IK_leg(uLINK(BODY), (-1)^stanceFoot*0.1, linkLength, ...
            linkLength, swingfoot);

        for j=0:5
            uLINK(stanceHipLink+j).q = qRStance(j+1);
            uLINK(swingHipLink+j).q = qRSwing(j+1);
        end
        ForwardKinematics(1);

        % graphing
        figure(robotViewH1)
        clf
        DrawRobot
        view([0,0])

        figure(robotViewH2)
        clf
        DrawRobot
        view([90,0])

        figure(robotViewH3)
        clf
        DrawRobot
        view([30,25])
        
        figure(graphPlotH)
        subplot(2,1,1)
        hold on
        plot((ii-1)*tsPerStep+i,xComRef((ii-1)*tsPerStep+i),'o')
        plot((ii-1)*tsPerStep+i,xZmp((ii-1)*tsPerStep+i),'x')
        legend('xComRef','xZmpRef','xZmp')
        subplot(2,1,2)
        hold on
        plot((ii-1)*tsPerStep+i,yComRef((ii-1)*tsPerStep+i),'o')
        plot((ii-1)*tsPerStep+i,yZmp((ii-1)*tsPerStep+i),'x')
        legend('yComRef','yZmpRef','yZmp')

        pause(0.001)
    end

    % appending steps that stay in the same place
    steps = [steps(:,2:end), [steps(1,end); 0]]

    stanceFoot = mod(stanceFoot + 1, 2);
end

% reinitialize for next steps
xErr = xq(1,tsPerStep*stepGroupSize+1 );
yErr = yq(1,tsPerStep*stepGroupSize+1);
xq0 = xq(:,tsPerStep*stepGroupSize+1);
yq0 = yq(:,tsPerStep*stepGroupSize+1);

%%%%%%%%%%%%
% Stopping %
%%%%%%%%%%%%

for stepGroupIdx = 1:2
    stepGroupIdx
    xZmpRef = footsteps(steps(1,:), tsPerStep);
    yZmpRef = footsteps(steps(2,:), tsPerStep);

    [xq, xZmp, ux] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
        xZmpRef, xq0(2:4), xErr, previewLength);
    [yq, yZmp, uy] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
        yZmpRef, yq0(2:4), yErr, previewLength);

    xComRef = xq(2,:);
    yComRef = yq(2,:);
    stanceFoot

    figure(graphPlotH)
    subplot(2,1,1)
    cla
    hold on
    plot(xComRef)
    plot(xZmpRef)
    plot(xZmp)
    legend('xComRef','xZmpRef','xZmp')
    subplot(2,1,2)
    cla
    hold on
    plot(yComRef)
    plot(yZmpRef)
    plot(yZmp)
    legend('yComRef','yZmpRef','yZmp')

    for ii = 1:2 % each step

        switch(stanceFoot)
            case(RIGHT_FOOT)
                stanceFootLink = RLEG_J5;
                stanceHipLink = RLEG_J0;
                swingHipLink = LLEG_J0;
                swingFootLink = LLEG_J5;
            case(LEFT_FOOT)
                stanceFootLink = LLEG_J5;
                stanceHipLink = LLEG_J0;
                swingHipLink = RLEG_J0;
                swingFootLink = RLEG_J5;
            otherwise
                disp 'stance foot isn''t being used correctly'
        end

        % come to rest with open stance, LEFT_FOOT is the starting foot atm
        footPlacementPair = [[uLINK(swingFootLink).p(1); ...
            uLINK(swingFootLink).p(2)] [steps(1,end); (-1)^stanceFoot*0.1]]; 

        [xSwing, ySwing, zSwing] = ...
            cubic_spline_trajectory( footPlacementPair(1,:), ...
            footPlacementPair(2,:), ...
            [GND, zSwingHeight], tsPerStep);

        for i=1:tsPerFrame:tsPerStep
            uLINK(BODY).p = [xComRef((ii-1)*tsPerStep+i), ...
                yComRef((ii-1)*tsPerStep+i), uLINK(BODY).p(3)]';
            uLINK(BODY).R = eye(3);

            % set stance foot
            stancefoot.p = [uLINK(stanceFootLink).p(1), ...
                uLINK(stanceFootLink).p(2), GND]';
            stancefoot.R = RPY2R([0, 0, 0]);  %  -pi/4 < q < pi/4
            qRStance = IK_leg(uLINK(BODY), (-1)^stanceFoot*-0.1, linkLength, ...
                linkLength, stancefoot);

            % set swing foot
            swingfoot.p = [xSwing(i), ySwing(i), zSwing(i)]';
            swingfoot.R = RPY2R([0, 0, 0]);
            qRSwing = IK_leg(uLINK(BODY), (-1)^stanceFoot*0.1, linkLength, ...
                linkLength, swingfoot);

            for j=0:5
                uLINK(stanceHipLink+j).q = qRStance(j+1);
                uLINK(swingHipLink+j).q = qRSwing(j+1);
            end
            ForwardKinematics(1);

            % graphing
            figure(robotViewH1)
            clf
            DrawRobot
            view([0,0])

            figure(robotViewH2)
            clf
            DrawRobot
            view([90,0])
            
            figure(robotViewH3)
            clf
            DrawRobot
            view([30,25])
            
            figure(graphPlotH)
            subplot(2,1,1)
            hold on
            plot((ii-1)*tsPerStep+i,xComRef((ii-1)*tsPerStep+i),'o')
            plot((ii-1)*tsPerStep+i,xZmp((ii-1)*tsPerStep+i),'x')
            legend('xComRef','xZmpRef','xZmp')
            subplot(2,1,2)
            hold on
            plot((ii-1)*tsPerStep+i,yComRef((ii-1)*tsPerStep+i),'o')
            plot((ii-1)*tsPerStep+i,yZmp((ii-1)*tsPerStep+i),'x')
            legend('yComRef','yZmpRef','yZmp')
            drawnow
            pause(0.001)
        end

        % reference zmp stays in the same place
        steps = [steps(:,2:end), [steps(1,end); ...
            0]]

        stanceFoot = mod(stanceFoot + 1, 2);
    end
    
    % reinitialize for next steps
    xErr = xq(1,tsPerStep*stepGroupSize+1 );
    yErr = yq(1,tsPerStep*stepGroupSize+1);
    xq0 = xq(:,tsPerStep*stepGroupSize+1);
    yq0 = yq(:,tsPerStep*stepGroupSize+1);
end

%TODO: Split all of the walking behaviors into functions so that they
% aren't all fractured everywhere

% TODO: Implement turning in the steps
