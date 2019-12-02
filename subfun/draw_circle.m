function [ output_args ] = draw_circle (x,y,r,color)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
if(nargin==3)
    color='-k';
end
t=0:0.01:2*pi;
X=x+r*cos(t);
Y=y+r*sin(t);
plot(X,Y,color,'LineWidth',1);
end

