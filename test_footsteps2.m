clc
close all
clear

deltax = .05
deltay = .1
dx = [0   deltax*ones(1,63) 0]
dy = []
for i=1:length(dx)
    dy(i) =(-1)^(-1*mod(i,2))*deltay
end
%%

R = @(th) [cos(th) -sin(th); sin(th) cos(th)] 


% xref = footsteps(xsteps,ST);
% yref = footsteps(ysteps,ST);
% hold on
% plot(pxref,pyref)
% plot(xref,yref)
% plot(xsteps,ysteps,'o')

% th = 0:.1:2*pi
% for i=1:length(th)
%     pos(:,i) = R(th(i))*[1;0]
% end
% plot(pos(1,:),pos(2,:),'o')

r = [0; 0];
th = 0
dth = .1*ones(1,65)
rlist = []
thlist = []
for i=1:length(dx)
    rlist(:,end+1) = r
    thlist(end+1) = th
    r = r + R(th+dth(i))*[dx(i);dy(i)]
    th = th + dth(i)
    
%     pL = r + R(th)*[0;ds]
%     pR = 

    
end

plot(rlist(1,:),rlist(2,:),'-o')

