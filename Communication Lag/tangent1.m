function [X1,X2,X3,X4]=tangent1(P1, P2, r, o)
%P1 is the origin of the agent
%P2 is the origin of the obstacle
%r is the radius of the agent
%o is radius  of the oi\bstacle
%P1 is the origin of the agent
%%THis file is used to calculate the slope and the end points of the
%%collision cone which is used for linearization
R=r+o;
P = P1-P2;
d2 = dot(P,P);
Q0 = P2+R^2/d2*P;
T = R/d2*sqrt(d2-R^2)*P*[0,1;-1,0];
Q1 = Q0+T;
Q2 = Q0-T;
m1=(Q1(2)-P1(2))/(Q1(1)-P1(1));
m2=(Q2(2)-P1(2))/(Q2(1)-P1(1));
a=7;
if(P1(1)>(P2(1)+R))
%fprintf("AAAA")    
F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
X1=F(-a, m1, P1);
X2=F(-o, m1, P1);
X3=F(-a, m2, P1);
X4=F(-o, m2, P1);

elseif(P1(1)>(P2(1)-R) && P1(1)<(P2(1)+R) && P1(2)>P2(2))
%fprintf("IIII")    
F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
X1=F(-a, m1, P1);
X2=F(-o, m1, P1);
X3=F(4.2, m2, P1);
X4=F(o, m2, P1);
elseif(P1(1)>(P2(1)-R) && P1(1)<(P2(1)+R)&& P1(2)<P2(2))
%fprintf("JJJJ")
F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
X1=F(a, m1, P1);
X2=F(o, m1, P1);
X3=F(-a, m2, P1);
X4=F(-o, m2, P1);
elseif(P1(1)==(P2(1)+R) || P1(1)==(P2(1)-R))
 %fprintf("A");   
 if(isinf(m1))   
    X1=P1+[0,sign(P2(2)-P1(2))*a];
    X2=P1+[0,sign(P2(2)-P1(2))*o];
 else
    F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
    X1=F(sign(P2(1)-P1(1))*a, m1, P1);
    X2=F(sign(P2(1)-P1(1))*o, m1, P1);  
 end
    
 if(isinf(m2))   
    X3=P1+[0,sign(P2(2)-P1(2))*a];
    X4=P1+[0,sign(P2(2)-P1(2))*o];
 else
    F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
    X3=F(sign(P2(1)-P1(1))*a, m2, P1);
    X4=F(sign(P2(1)-P1(1))*o, m2, P1);  
 end
    
else
%fprintf("XXXX")    
F= @(d,m,X) X + d*[1, m]/sqrt(1 + m^2);
X1=F(a, m1, P1);
X2=F(o, m1, P1);
X3=F(a, m2, P1);
X4=F(o, m2, P1);
end

end
