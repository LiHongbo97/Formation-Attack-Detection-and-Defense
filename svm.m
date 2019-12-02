function label_out = svm(lmd)
clc 
load('data.mat')
load('attmse.mat')
label = a(:,100);
instance = a(:,1:99);
instance = sparse(instance);
libsvmwrite('libsvm_data.mat',label, instance); 
pre_label = attmse(:,100);
pre_att = sparse(attmse(1:5,1:99));
[data_label,data_instance]=libsvmread('libsvm_data.mat');  
model = svmtrain(data_label(1:60),data_instance(1:60,:),'-s 4 -t 2 -c 1 -g 0.1');
% [predict_label,accuracy,dec_values] = svmpredict(data_label(21:25),data_instance(21:25,:),model);
[predict_label,accuracy,dec_values] = svmpredict(pre_label(1:5),pre_att,model);
label_out=predict_label;
end
