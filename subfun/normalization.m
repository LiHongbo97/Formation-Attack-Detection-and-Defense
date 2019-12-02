function [ feature_new ] = normalization( feature ,mi,ma,lower,upper)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
feature_new=lower+(upper-lower)*(feature-mi)/(ma-mi);
end

