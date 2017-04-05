function [asol,bsol,csol,dsol] = cubic_spline(x0,x1,y0,y1,yp0,yp1)
% CUBIC_SPLINE Creates a cubic spline
% Based on two points, creates a smooth spline with a continuous second
% derivative
% Daniel Sun 3-20-17


syms a b c d;

eqns = [a*x0^3 + b*x0^2 + c*x0 + d == y0,
       a*x1^3 + b*x1^2 + c*x1 + d == y1,
       3*a*x0^2 + 2*b*x0 + c == yp0,
       3*a*x1^2 + 2*b*x1 + c == yp1];

[asol,bsol,csol,dsol] = solve(eqns,[a,b,c,d]);
end