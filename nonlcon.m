function [c,ceq] = nonlcon(u,a,x0,xf,dt)
 c=[];
 x=x0;
xd=0;
res=0;
for i=1:length(u(:,1))
    xdd=a.forward(x,u(:,i));
    xd=xd+xdd*dt;
    x=x+[xd*dt;xdd*dt];
    res=res+norm(u(:,i));
end
ceq=norm(x(end,:)-xf);