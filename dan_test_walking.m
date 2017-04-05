% Prototype walking script for humanoid robot

clc
close all

% addpath(strcat(pwd(),'/preview_control'))
% addpath(strcat(pwd(),'/graphing'))


% needs the file regen.m
if exist('robot_config.mat','file') ~= 2 || REGENERATE_ROBOT_DATA == true
    disp 'regenerating robot preview controller'
    REGENERATE_ROBOT_DATA = false;
    SetupBipedRobot3
    walk_config;

    % defining the zmp system. Same for x and y
    T = 0.005;
    t = 0:T:100000;
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

    steps = [ 0 0     .2    .4  .6 .8   1  1.2;
              -.1 .1  -0.1  0.1 -.1 .1 -.1 .1];

    tsPerStep = round(previewLength/1.5);
    xStepSize = .2;
    yStepSize = .1;
    stepIdx = 1;
    xErr = 0;
    yErr = 0;
    zSwingHeight = 0.15;
    RIGHT_FOOT = 0;
    LEFT_FOOT = 1;
    stanceFoot = LEFT_FOOT;
    stepGroupSize = 4;

    xq = [0 uLINK(BODY).p(1), 0 0]';
    yq = [0 uLINK(BODY).p(2), 0 0]';

    save('robot_config')
else
    disp 'got here'
    load('robot_config.mat')
end

robotViewHandle1 = figure('units','normalized','position',[0 0.5 0.3 0.3]);
robotViewHandle2 = figure('units','normalized','position',[0.32 0.5 0.3 0.3]);
graphPlotHandle = figure('units','normalized','position',[0 0.1 0.3 0.3]);


%%%%%%%%%%%
% Walking %
%%%%%%%%%%%
for stepGroupIdx = 1:2 % group of steps
    stepGroupIdx
    xZmpRef = footsteps(steps(1,:), tsPerStep);
    yZmpRef = footsteps(steps(2,:), tsPerStep);
    
    [xq, xZmp, ux] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
        xZmpRef, xq(2:4,end), xErr, previewLength);
    [yq, yZmp, uy] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
        yZmpRef, yq(2:4,end), yErr, previewLength);
    
    xComRef = xq(2,:);
    yComRef = yq(2,:);
    stanceFoot

    figure(graphPlotHandle)
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
                                     [0, zSwingHeight], tsPerStep);
                                 
        for i=1:5:tsPerStep
            uLINK(BODY).p = [xComRef((ii-1)*tsPerStep+i), ...
                          yComRef((ii-1)*tsPerStep+i), uLINK(BODY).p(3)]';
            uLINK(BODY).R = eye(3);

            % set stance foot
            stancefoot.p = [uLINK(stanceFootLink).p(1), ...
                            uLINK(stanceFootLink).p(2), 0]';

            stancefoot.R = RPY2R([0, 0, 0]);  %  -pi/4 < q < pi/4
            qRStance = IK_leg(uLINK(BODY), (-1)^stanceFoot*-0.1, 0.4, ...
                0.4, stancefoot);

            % set swing foot
            swingfoot.p = [xSwing(i), ySwing(i), zSwing(i)]';
            swingfoot.R = RPY2R([0, 0, 0]);
            qRSwing = IK_leg(uLINK(BODY), (-1)^stanceFoot*0.1, 0.4, ...
                0.4, swingfoot);

            for j=0:5
                uLINK(stanceHipLink+j).q = qRStance(j+1);
                uLINK(swingHipLink+j).q = qRSwing(j+1);
            end
            ForwardKinematics(1);
            
            % graphing
            figure(robotViewHandle1)
            clf
            DrawRobot
            view([30,10])
            
            figure(robotViewHandle2)
            clf
            DrawRobot
            view([90,10])
            
            figure(graphPlotHandle)
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

        steps = [steps(:,2:end), [steps(1,end) + xStepSize; ...
                 (-1)^stanceFoot*yStepSize]]

        stanceFoot = mod(stanceFoot + 1, 2);
    end
    
    xErr = xq(1,tsPerStep*stepGroupSize)
    yErr = yq(1,tsPerStep*stepGroupSize)
end
% TODO: have the robot be able to oscillate back and forth
% Simulate dynamics????
% The robot needs to be able move forward
% TODO: the stance foot is getting warped into position; should be at the 
% altered position??? It shouldn't be changing regardless
% 3-29-17 the reference zmp and the footstep thing needs to match up

