function [ mse ] = cal_mse(pose,ideal_pose)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
mse = 0;
mse = ((pose(1)-ideal_pose(1))^(2)+(pose(2)-ideal_pose(2))^(2))^(1/2);

end


