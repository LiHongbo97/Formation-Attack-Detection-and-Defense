%%%for ob_data.m
%%%ÓÃÓÚ¹¥»÷²âÊÔ
clear all;
load('model_x.mat');
load('model_y.mat');
goal=[15 15];
init_f=[-7 -3 0; %%%[x y th]
    -4 -2 0;
    -5 -4 pi/4; 
    -8 -3 -pi/4;
    -2 -2 pi/4];  

attacker=[3 3.3];
 direct_attack( goal,attacker,init_f,gprMdl,gprMd2);