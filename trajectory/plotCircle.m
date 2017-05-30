function h_circle = plotCircle(x,y,r,ang_start,ang_end)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=ang_start:0.01:ang_end; 
%ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
h_circle = plot(x+xp,y+yp);
end