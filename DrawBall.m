function DrawBall(p,r)
%DRAWBALL Draws a ball centered at the specified coordinates x0,y0,z0
[x,y,z] = sphere;
hSurface = surf(r*x+p(1),r*y+p(2),r*z+p(3),[1,1,1]);
set(hSurface,'FaceColor',[1 0 0],'FaceAlpha',0.5);
end

