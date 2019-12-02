function interpoint( x1,y1,x2,y2,x3,y3,colo,lstyle)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
if (nargin==6)
    colo='k';
    lstyle=':';
end
syms k b m n x y;
if(x1==x2)%x1x2直线斜率不存在
    solx=x1;
    soly=y3;
elseif(y1==y2)%x1x2直线斜率为0
    solx=x3;
    soly=y1;
else 
    solk=(y2-y1)/(x2-x1);
    solb=y2-solk*x2;
    solk1=-1/solk;
    solb1=y3-solk1*x3;
    solx=(solb1-solb)/(solk-solk1);
    soly=solk*solx+solb;
%     [solx,soly] = solve(solk1*x-y+solb1==0,solk*x-y+solb==0,x,y);
end
line([x1,solx],[y1,soly],'color',colo,'linestyle',lstyle);
line([x3,solx],[y3,soly],'color',colo,'linestyle',lstyle);
end

