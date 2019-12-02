function [ result ] = seek_ang( th1,th2 )
%UNTITLED4 弧度制
%   th1为要转到的角、th2为当前位置的角度
%  th1-th2即为相对参考点主轴的夹角

result=th1-th2;
if(th1>0)
    if(result>pi)
        result=result-2*pi;
    end
else
    if(result<-pi)
        result=result+2*pi;
    end
end


% if(th1*th2>=0)
%     result=th1-th2;
% else
%     if(th1>0)
%         result=th1-th2;
%         if(result>pi)
%             result=result-2*pi;
%         end
%     else
%         result=th1-th2;
%         if(result<-pi)
%             result=result+2*pi;
%         end
%     end
% end

end

