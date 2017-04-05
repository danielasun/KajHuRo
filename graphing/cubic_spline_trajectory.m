function [xsol,ysol,zsol] = cubic_spline_trajectory(xref,yref,zref,nsteps)
% FUNCTION cubic_spline_trajectory
% traj = cubic_spline_trajectory(xref,yref,zref,nsteps)
% Creates a 3d cubic spline trajectory using the cubic_spline function and
% returns them all as xsol, ysol, zsol
% xref, yref, zref are x,y,z endpoints to go through
% xref and yref just go straight through, zref goes up and goes back down.
t1 = 1:nsteps/2;
t2 = nsteps/2+1:nsteps;
t = [t1 t2];

% x trajectory
[a1sol,b1sol,c1sol,d1sol] = cubic_spline(0,nsteps,xref(1),xref(2),0,0);
xsol = double([a1sol*t.^3+b1sol*t.^2+c1sol*t+d1sol]);

% y trajectory
[a1sol,b1sol,c1sol,d1sol] = cubic_spline(0,nsteps,yref(1),yref(2),0,0);
ysol = double([a1sol*t.^3+b1sol*t.^2+c1sol*t+d1sol]);

% ztrajectory
[a1sol,b1sol,c1sol,d1sol] = cubic_spline(0,nsteps/2,zref(1),zref(2),0,0);
[a2sol,b2sol,c2sol,d2sol] = cubic_spline(nsteps/2+1,nsteps,zref(2),zref(1),0,0);
zsol = double([a1sol*t1.^3+b1sol*t1.^2+c1sol*t1+d1sol, a2sol*t2.^3+b2sol*t2.^2+c2sol*t2+d2sol]);



