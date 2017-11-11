clear all
close all

l1 = 2;
l2 = 2;
l3 = 2;

t = 1;



figure(1)

%% generate benchmark data 
for i = 0:0.0157:pi;
   
theta1 = i;
theta2 = i;
%% theta1+theta2+theta3=pi
theta3 = pi-2*i;

theta_total(t)=theta1+theta2+theta3;

px(t) = l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);
py(t) = l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);

l1x(t) = l1*cos(theta1);
l1y(t) = l1*sin(theta1);

l2x(t) = l1*cos(theta1)+l2*cos(theta2+theta1);
l2y(t) = l1*sin(theta1)+l2*sin(theta2+theta1);

l3x(t) = l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);   
l3y(t) = l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);

plot(px,py,'r*')

line([l2x(t); l3x(t)],[l2y(t); l3y(t)]);
drawnow;
t = t+1;
    
end

%%

figure(2)
for i=1:t-1
    
[theta1, theta2, theta3]=IK_analytical_func(px(i), py(i),theta_total(i),l1,l2,l3);    

%%forward kinematics to compute the locations of the links for display
pxe(i) = l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);
pye(i) = l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);
    
l1x(i) =   l1*cos(theta1);
l1y(i) =   l1*sin(theta1);

l2x(i) =   l1*cos(theta1)+l2*cos(theta2+theta1);
l2y(i) =   l1*sin(theta1)+l2*sin(theta2+theta1);

l3x(i) =  l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);   
l3y(i) =  l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);

plot(pxe,pye,'b*')
line([0 l1x(i)],[0 l1y(i)],'LineWidth',4,'Color',[1 0 0])
line([l1x(i); l2x(i)],[l1y(i); l2y(i)],'LineWidth',4,'Color',[0 1 0])
line([l2x(i); l3x(i)],[l2y(i); l3y(i)],'LineWidth',4,'Color',[0 0 1])
drawnow;
pause(0.01);
    
end

%%



%%Jacobian based inverse kinematics, the current joint angles,theta1,theta2,theta3 need to be known

theta1 = 0.01;
theta2 = 0.01;
theta3 = 0.01;
%try different theta3 value to make the IK work better


pxk = l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);
pyk = l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);
theta_totalk=theta1+theta2+theta3;

figure(3)

for i=1:t-1
    
    
[theta1,theta2,theta3]=IK_Jacobian_func(px(i),py(i),pxk,pyk,theta_total(i),theta_totalk,theta1,theta2,theta3,l1,l2,l3);

%%updating theta_total for Jacobian matrix
theta_totalk=theta1+theta2+theta3;

%%forward kinematics to updating the locations of links for Jacobian matrix
pxk = l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);
pyk = l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);

pxj(i)=pxk;
pyj(i)=pyk;

l1x(i) =   l1*cos(theta1);
l1y(i) =   l1*sin(theta1);

l2x(i) =   l1*cos(theta1)+l2*cos(theta2+theta1);
l2y(i) =   l1*sin(theta1)+l2*sin(theta2+theta1);

l3x(i) =  l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);   
l3y(i) =  l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);

plot(pxj,pyj,'g*')
line([0 l1x(i)],[0 l1y(i)],'LineWidth',4,'Color',[1 0 0])
line([l1x(i); l2x(i)],[l1y(i); l2y(i)],'LineWidth',4,'Color',[0 1 0])
line([l2x(i); l3x(i)],[l2y(i); l3y(i)],'LineWidth',4,'Color',[0 0 1])
drawnow;
pause(0.01);

end



%error for the analytical inverse kinematics
errorx=px-pxe;
errory=py-pye;


%error for Jacobian based inverse kinematics
errorxj=px-pxj;
erroryj=py-pyj;



figure(4)
plot(px,py,'r*')
hold on
plot(pxe,pye,'b.')
plot(pxj,pyj,'g.')
title('trajectory','FontSize',14)

figure(5)
plot(errorx,'b-')
hold on
plot(errory,'r-')
title('Error from analytical IK','FontSize',14)


figure(6)
plot(errorxj,'b-')
hold on
plot(erroryj,'r-')
title('Error from Jacobian fullrank IK','FontSize',14)

