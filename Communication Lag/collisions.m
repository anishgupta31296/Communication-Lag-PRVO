function v=collisions(p1,p2,v1,v2,R)
pr=p1-p2;
vr=v1-v2;
cone=(norm(vr)^2)*(R^2 - norm(pr)^2) + (pr(1)*vr(1) + pr(2)*vr(2))^2
if(cone<0)
    v=0
else
    v=1;
end
end