function [c,ceq]=const(b_list,n_list,control)
 c=[];
 for i=1:length(b_list)
  c(i)=n_list(i,1)*control(1)+n_list(i,2)*control(2)-b_list(i); 
 end
 ceq=[];
end
