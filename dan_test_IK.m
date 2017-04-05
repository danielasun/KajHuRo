clc
close all
clear

addpath(strcat(pwd(),'/preview_control'))
addpath(strcat(pwd(),'/graphing'))

SetupBipedRobot3

LEFT_FOOT = 1;
RIGHT_FOOT = 0;
stanceFoot = RIGHT_FOOT;
stancefoot.p = [0, -.1, 0]'
stancefoot.R = RPY2R([0, 0, 0]);  %  -pi/4 < q < pi/4
qRStance = IK_leg(uLINK(BODY), (-1)^stanceFoot*-0.1, 0.4, 0.4, stancefoot);

for j=0:5
    if stanceFoot == RIGHT_FOOT
        uLINK(RLEG_J0+j).q = qRStance(j+1);
    end
    if stanceFoot == LEFT_FOOT
        uLINK(LLEG_J0+j).q = qRStance(j+1);
    end
end

ForwardKinematics(1)
DrawRobot
view([270,0])