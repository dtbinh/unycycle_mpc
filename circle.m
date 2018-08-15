function [x,y] = circle(R,cx,cy)
%%%%%%%%%%%%%%%%%%%
% 画圆函数
%%%%%%%%%%%%%%%%%%%
nb_pts = 100;
alpha=0:pi/nb_pts:2*pi;%角度[0,2*pi]
%R=2;%半径
x=R*cos(alpha)+cx;
y=R*sin(alpha)+cy;
plot(cx,cy,'r+',x,y);
grid on;
axis equal;