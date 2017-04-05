clc
clear
close all

syms a b c d
x0 = 5
y0 = 6
x1 = 100
y1 = -19

% eqns = [a*x0^3 + b*x0^2 + c*x0 + d == y0,
%        a*x1^3 + b*x1^2 + c*x1 + d == y1,
%        3*a*x0^2 + 2*b*x0 + c == 0,
%        3*a*x1^2 + 2*b*x1 + c == 0]
% 
% [asol,bsol,csol,dsol] = solve(eqns,[a,b,c,d])

[asol,bsol,csol,dsol] = cubic_spline(x0,x1,y0,y1,-1,0)


t = linspace(x0,x1,20);
hold on
plot(t,asol*t.^3+bsol*t.^2+csol*t+dsol)
plot(x0,y0,'ro')
plot(x1,y1,'ro')