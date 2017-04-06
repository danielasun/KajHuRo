%%% SetupBipedRobot3.m
%%% Altering some of Kajita's original stuff, lengthening the legs and
%%% using inverse kinematics to set up the original resting pose.
global uLINK

ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

linkLength = .35;
comHeight = .7;
GND = 0.109115348192675


uLINK    = struct('name','BODY'    , 'm', 10, 'sister', 0, 'child', 2, 'b',[0  0   comHeight]','a',UZ,'q',0);

uLINK(2) = struct('name','RLEG_J0' , 'm',  5, 'sister', 8, 'child', 3, 'b',[0 -0.1 0]'   ,'a',UZ,'q',0);
uLINK(3) = struct('name','RLEG_J1' , 'm',  1, 'sister', 0, 'child', 4, 'b',[0  0   0]','a',UX,'q',0);
uLINK(4) = struct('name','RLEG_J2' , 'm',  5, 'sister', 0, 'child', 5, 'b',[0  0   0]'   ,'a',UY,'q',0);
uLINK(5) = struct('name','RLEG_J3' , 'm',  1, 'sister', 0, 'child', 6, 'b',[0  0  -linkLength]' ,'a',UY,'q',0);
uLINK(6) = struct('name','RLEG_J4' , 'm',  6, 'sister', 0, 'child', 7, 'b',[0  0  -linkLength]' ,'a',UY,'q',0);
uLINK(7) = struct('name','RLEG_J5' , 'm',  2, 'sister', 0, 'child', 0, 'b',[0  0   0  ]' ,'a',UX,'q',0);

uLINK(8) = struct('name','LLEG_J0' , 'm',  5, 'sister', 0, 'child', 9, 'b',[0  0.1 0]'   ,'a',UZ,'q',0);
uLINK(9) = struct('name','LLEG_J1' , 'm',  1, 'sister', 0, 'child',10, 'b',[0  0   0]','a',UX,'q',0);
uLINK(10)= struct('name','LLEG_J2' , 'm',  5, 'sister', 0, 'child',11, 'b',[0  0   0]'   ,'a',UY,'q',0);
uLINK(11)= struct('name','LLEG_J3' , 'm',  1, 'sister', 0, 'child',12, 'b',[0  0  -linkLength]' ,'a',UY,'q',0);
uLINK(12)= struct('name','LLEG_J4' , 'm',  6, 'sister', 0, 'child',13, 'b',[0  0  -linkLength]' ,'a',UY,'q',0);
uLINK(13)= struct('name','LLEG_J5' , 'm',  2, 'sister', 0, 'child', 0, 'b',[0  0   0  ]' ,'a',UX,'q',0);

FindMother(1);   %?@��?C�������N��?������Ƃɕꃊ���N��?ݒ肷��

%% �t�B?[���h�̒ǉ�
for n=1:length(uLINK)
    uLINK(n).dq     = 0;            % ��?ߑ��x   [rad/s]
    uLINK(n).ddq    = 0;            % ��?߉����x [rad/s^2]
    uLINK(n).c      = [0 0 0]';     % ?d?S�ʒu   [m]
    uLINK(n).I      = zeros(3,3);   % ?d?S���̊�?��e���\�� [kg.m^2]
    uLINK(n).Ir     = 0.0;          % ��?[�^�̓d�C�q��?���?[�?���g [kg.m^2]
    uLINK(n).gr     = 0.0;          % ��?[�^�̌����� [-]
    uLINK(n).u      = 0.0;          %  ��?߃g���N [Nm]
end

%%% �v�?�O���������₷�����邽��?C�����Nname�Ɠ����̕�?���ID��?����Z�b�g
for n=1:length(uLINK)
    eval([uLINK(n).name,'=',num2str(n),';']);
end

%%%%%%%%%?@����?C����ё�����?��̂Ń��f���� %%%%%%%%%
shape = [0.1 0.3 0.5];     % ��?s��?C�??C?��� [m]
com   = [0 0 0.3]';        % ?d?S�ʒu
SetupRigidBody(BODY, shape,com);

shape = [0.2 0.1 0.02];    % ��?s��?C�??C?��� [m]
com   = [0.05  0 -0.04]';   % ?d?S�ʒu
SetupRigidBody(RLEG_J5, shape,com);

shape = [0.2 0.1 0.02];     % ��?s��?C�??C?��� [m]
com   = [0.05  0 -0.04]';    % ?d?S�ʒu
SetupRigidBody(LLEG_J5, shape,com);

%%%%%%%%%%% ����َp?��ֈ�?s?@%%%%%%%%%%%%
uLINK(RLEG_J2).q = -10.0*ToRad;
uLINK(RLEG_J3).q = 20.0*ToRad;
uLINK(RLEG_J4).q = -10.0*ToRad;

uLINK(LLEG_J2).q = -10.0*ToRad;
uLINK(LLEG_J3).q = 20.0*ToRad;
uLINK(LLEG_J4).q = -10.0*ToRad;

uLINK(BODY).p = [0.0, 0, comHeight]';
uLINK(BODY).R = eye(3);

ForwardKinematics(1);

% Added by Daniel
Rfoot.p = [0,-.1, GND]';
Rfoot.R = RPY2R([0,0,0]);  %  -pi/4 < q < pi/4
qR2 = IK_leg(uLINK(BODY),-0.1,linkLength,linkLength,Rfoot);

% set left foot
Lfoot.p = [0,.1, GND]';
Lfoot.R = RPY2R([0,0,0]);  %  -pi/4 < q < pi/4
qR3 = IK_leg(uLINK(BODY),0.1,linkLength,linkLength,Lfoot);
for n=0:5
    uLINK(RLEG_J0+n).q = qR2(n+1);
    uLINK(LLEG_J0+n).q = qR3(n+1);
end


ForwardKinematics(1);