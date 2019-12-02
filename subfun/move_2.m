function [x2,y2,heading2 ] = move_2( x1,y1,heading,ang,d)
%前三项为位姿信息，ang为障碍距离中心轴的夹角，delta_ang和d用来决定偏转角和距离
%   delta_ang可设30度
heading2=heading+ang;
 x2=x1+d*cos(heading2);
 y2=y1+d*sin(heading2);
end


