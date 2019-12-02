function x = motion(x, u,dt)
% Motion Model
% x=[x;y;th]  u = [vt; wt];%%当前时刻的速度、角速度

delta_th=u(2)*dt;
delta_x=u(1)*dt*cos(x(3)+delta_th/2);
delta_y=u(1)*dt*sin(x(3)+delta_th/2);
x(1)=x(1)+delta_x;
x(2)=x(2)+delta_y;
x(3)=x(3)+delta_th;
% %% 近似模型
% delta_th=u(2)*dt;
% delta_x=u(1)*dt*cos(x(3)+delta_th);
% delta_y=u(1)*dt*sin(x(3)+delta_th);
% x(1)=x(1)+delta_x;
% x(2)=x(2)+delta_y;
% x(3)=x(3)+delta_th;
end




