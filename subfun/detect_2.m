function [ in,ang,r] = detect_2( x1,y1,x2,y2,th,threld_th,radius )
%th为点1的方位角
%threld_th,radius分别扇形扫描角和扫描半径
%ang为障碍物距离主轴的夹角
%%判断一个点是否落在扫描区域内
r=sqrt((x2-x1)^2+(y2-y1)^2);
t=atan2(y2-y1,x2-x1); %%%弧度值
%%%求角度差
ang= seek_ang(t,th);
if(r<=radius)
    %%ang=angle(x1,y1,x2,y2)-th;
    if(abs(ang)<=threld_th)
        in=1;
    else
        in=0;
    end
else
    in=0;
end



