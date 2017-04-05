% Prototype walking script for humanoid robot

clc
close all
clear

addpath(strcat(pwd(),'/preview_control'))
addpath(strcat(pwd(),'/graphing'))

REGENERATE_ROBOT_DATA = 0;


if exist('robot_config.mat','file') ~= 2 || REGENERATE_ROBOT_DATA == true
    disp 'regenerating robot preview controller'
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

    steps = [ 0   0    0    .3 ;
              0  0  -0.1  0.1];

    tsPerStep = previewLength/2;

    stepIdx = 1;
    xErr = 0;
    yErr = 0;
    zSwingHeight = 0.15;
    RIGHT_FOOT = 0;
    LEFT_FOOT = 1;
    stanceFoot = LEFT_FOOT;

    xq = [0 uLINK(BODY).p(1), 0 0]';
    yq = [0 uLINK(BODY).p(2), 0 0]';

    save('robot_config')
else
    disp 'got here'
    load('robot_config.mat')
end

robotViewHandle = figure('units','normalized','position',[0 0.5 0.3 0.3])
graphPlotHandle = figure('units','normalized','position',[0 0.1 0.3 0.3])


%%%%%%%%%%%
% Walking %
%%%%%%%%%%%
for iii = 1:3 % group of steps
    
    
    for ii = 1:3
        xZmpRef = footsteps(steps(1,:), tsPerStep);
        yZmpRef = footsteps(steps(2,:), tsPerStep);

        [xq, xZmp, ux] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
            xZmpRef, xq(2:4,end), xErr, previewLength);
        [yq, yZmp, uy] = SimulatePreviewDynamics(sys, sys_t, Gi, Gx, Gd, ...
            yZmpRef, yq(2:4,end), yErr, previewLength);

        xComRef = xq(2,:);
        yComRef = yq(2,:);
        stanceFoot
        switch(stanceFoot)
            case(RIGHT_FOOT)
                footPlacementPair = [[uLINK(LLEG_J5).p(1); ...
                                      uLINK(LLEG_J5).p(2)] steps(:,1)]
                stanceFootLink = 7;
            case(LEFT_FOOT)
                footPlacementPair = [[uLINK(RLEG_J5).p(1); ...
                                      uLINK(RLEG_J5).p(2)] steps(:,1)]
                stanceFootLink = 13;
            otherwise
                disp 'stance foot isn''t being used correctly'
        end

        [xSwing, ySwing, zSwing] = ...
            cubic_spline_trajectory( footPlacementPair(1,:), ...
                                     footPlacementPair(2,:), ...
                                     [0, zSwingHeight], tsPerStep);

        figure(graphPlotHandle)
        subplot(2,1,1)
        cla
        hold on
        plot(xSwing)
        plot(xComRef)
        plot(xZmpRef)
        plot(xZmp)
        legend('xSwing','xComRef','xZmpRef','xZmp')
        subplot(2,1,2)
        cla
        hold on
        plot(yComRef)
        plot(yZmpRef)
        plot(ySwing)
        plot(yZmp)
        legend('ySwing','yComRef','yZmpRef','yZmp')

                                 
        for i=1:20:tsPerStep
            uLINK(BODY).p = [xComRef(i), yComRef(i), uLINK(BODY).p(3)]';
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
                switch(stanceFoot)
                    case(RIGHT_FOOT)
                        uLINK(RLEG_J0+j).q = qRStance(j+1);
                        uLINK(LLEG_J0+j).q = qRSwing(j+1);
                    case(LEFT_FOOT)
                        uLINK(RLEG_J0+j).q = qRSwing(j+1);
                        uLINK(LLEG_J0+j).q = qRStance(j+1);
                    otherwise
                        disp 'stance foot isn''t being used correctly'
                end
            end
            ForwardKinematics(1);
            
            figure(robotViewHandle)
            clf
            DrawRobot
            view([30,10])
            pause(0.001)
        end

        steps = [steps(:,2:end), [steps(1,end) + .2; (-1)^stanceFoot*.1]]

        xErr = xq(1,tsPerStep)
        yErr = yq(1,tsPerStep)

    %     stepIdx = stepIdx + 1;
        stanceFoot = mod(stanceFoot + 1, 2);
    end
end
% TODO: have the robot be able to oscillate back and forth
% Simulate dynamics????
% The robot needs to be able move forward
% TODO: the stance foot is getting warped into position; should be at the 
% altered position??? It shouldn't be changing regardless
% 3-29-17 the reference zmp and the footstep thing needs to match up

