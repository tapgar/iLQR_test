a=ArmModel;
uArrayInitial=zeros(2,100);
xf=[pi/2;pi/2;0;0];
x0=[0;0;0;0];
dt=0.01;
options = optimoptions('fmincon','MaxFunctionEvaluations',30000);
uArray=fmincon(@(u)fun(u,a,x0,xf,dt),uArrayInitial,[],[],[],[],[],[],@(u)nonlcon(u,a,x0,xf,dt),options);
