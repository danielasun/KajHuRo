%%%  robot_simulation.m
%%%  ’PˆÊƒxƒNƒgƒ‹–@‚É‚æ‚é“®—ÍŠwƒVƒ~ƒ…ƒŒ?[ƒVƒ‡ƒ“
%%%  ’??F?°–Ê‚Æ‚Ì?Ú?G”½—Í‚ðŒvŽZ‚µ‚Ä‚¢‚È‚¢‚Ì‚ÅŽ©—R—Ž‰º‚·‚é‚¾‚¯‚Å‚·?D
close all
clear

global uLINK G
G = 0;  % ?d—Í‰Á‘¬“x [m/s^2]

SetupBipedRobot2;   % ?}2.19?C?}2.20‚Ì2‘«•à?sƒ?ƒ{ƒbƒg‚Ìƒf?[ƒ^‚ð?Ý’è?i?d?SˆÊ’u?CŠµ?«ƒeƒ“ƒ\ƒ‹•t?j

%%%%%% “·‘Ì‚Ì‹óŠÔ‘¬“x?E‰Á‘¬“x %%%%%
uLINK(BODY).vo = [0, 0, 0]';
uLINK(BODY).w  = [0, 0, 0]';

uLINK(BODY).dvo = [0, 0, 0]';
uLINK(BODY).dw  = [0, 0, 0]';

%%%%%%%?@ŠÖ?ßƒgƒ‹ƒN %%%%%%%
u_joint = zeros(length(uLINK),1); % ‘SŠÖ?ßƒgƒ‹ƒN‚ð0‚É?Ý’è
u_joint(RLEG_J2) = -10.0;   % ‰EŒÒŠÖ?ßƒsƒbƒ`Ž²‚Éƒgƒ‹ƒN‚ð—^‚¦‚é?@[Nm]

%%%%%%% ƒVƒ~ƒ…ƒŒ?[ƒVƒ‡ƒ“ %%%%%%%
global Dtime

Dtime = 0.005;
EndTime = 0.3;
time = 0:Dtime:EndTime;
tsize = length(time);
com_m = zeros(tsize,3);

figure
tic
for k = 1:tsize
    %fprintf('time=%f\n',time(n))
    ForwardDynamics; 
    
    IntegrateEuler(1);    
    com = calcCoM;
    com_m(k,:) = com';
    
    hold off
    newplot
    DrawAllJoints(1);
    axis equal
    set(gca,...
        'CameraPositionMode','manual',...
        'CameraPosition',[4,4,1],...
        'CameraViewAngleMode','manual',....
        'CameraViewAngle',15,...
        'Projection','perspective',... 
        'XLimMode','manual',...
        'XLim',[-0.5 0.5],...
        'YLimMode','manual',...
        'YLim',[-0.5 0.5],...
        'ZLimMode','manual',...
        'ZLim',[0 1.5])
    grid on
    text(0.5, -0.4, 1.4, ['time=',num2str(time(k),'%5.3f')])

    drawnow;
%    if k == 1
%      Mov = moviein(tsize);
%    end
%    Mov(:,k) = getframe;   
end        
toc
