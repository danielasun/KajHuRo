clc
close all
clear

xsteps = [0 0 .3 .6 .9 1.2 1.5 1.8 1.8];
ysteps = [0 .1 -.1 .1 -.1 .1 -.1 .1 0];

T = .005;
xstep = .3;
ystep = .1;
chunk = ones(1,.8/T); % chunks of walking so that it's easy for me to make the trajectory
ST = length(chunk); % step time

pxref = [0*chunk 0*chunk 0*chunk 1*xstep*chunk 2*xstep*chunk 3*xstep*chunk 3*xstep*chunk 3*xstep*chunk 3*xstep*chunk ];
pyref = [0*chunk 0*chunk ystep*chunk -ystep*chunk ystep*chunk -ystep*chunk 0*chunk 0*chunk 0*chunk];


xref = footsteps(xsteps,ST);
yref = footsteps(ysteps,ST);
hold on
plot(pxref,pyref)
plot(xref,yref)
plot(xsteps,ysteps,'o')