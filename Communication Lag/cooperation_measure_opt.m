function [b_ij,n_star]=cooperation_measure_opt(vi,vj,cost_list,u_list,n,n2,cost_full)
 b_func=@(lambda,b_lij,n) lambda*b_lij+n*((1-lambda)*vi+lambda*vj)';
 lambda_list=0:0.01:1;
 b_list1=[];
 b_list2=[];
 for i=1:length(lambda_list)
  b_list1(i)=b_func(lambda_list(i),0,n);
  b_list2(i)=b_func(lambda_list(i),0,n2);
 end

 coop_list=[];
 coop_list2=[];
 for i=1:length(b_list1)
     indices=find(n*u_list'<b_list1(i));
     if(indices)
      f_u=u_list(indices);    
      coop_list(i)=(sum(cost_list(indices)))*(u_list(2,1)-u_list(1,1))/cost_full;
     else
      coop_list(i)=0;   
     end
      indices=find((-n*u_list')<(-b_list1(i)));
      if(indices)
       f_u=u_list(indices);    
       coop_list(i)=coop_list(i)+sum(cost_list(indices))*(u_list(2,1)-u_list(1,1))/cost_full;
      end
      
%      indices=find(n2*u_list'<b_list2(i));
%      if(indices)
%       f_u=u_list(indices);    
%       coop_list2(i)=sum(cost_list(indices))*(u_list(2,1)-u_list(1,1))/cost_full;
%      else
%       coop_list2(i)=0;   
%      end
%       indices=find((-n2*u_list')<(-b_list2(i)));
%       if(indices)
%        f_u=u_list(indices);    
%        coop_list2(i)=coop_list2(i)+sum(cost_list(indices))*(u_list(2,1)-u_list(1,1))/cost_full;
%       end
            
 end                     
 [v,i]=max(coop_list);
%  [v2,i2]=max(coop_list2);
 
 n_star=n;
 b_ij=b_list1(i);
%  if(v2>v)
% 
%   v=v2;
%   i=i2;
%   n_star=n2;
%   b_ij=b_list2(i);
%  end
 
end
