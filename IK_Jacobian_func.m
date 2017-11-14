function [theta1f,theta2f,theta3f]=IK_Jacobian_func(px,py,pxk,pyk,theta_total,theta_totalk,theta1,theta2,theta3,l1,l2,l3)
%pxk, pyk, theta_totalK are the curret position and orientation        
pv = [px-pxk py-pyk theta_total-theta_totalk];

q = [(-l1*sin(theta1)-l2*sin(theta2+theta1)-l3*(theta3+theta1)) -l2*sin(theta2+theta1)-(l3*sind(theta3+theta1)) -l3*sin(theta3+theta1);
    l1*cosd(theta1)+l2*cosd(theta2+theta1)+l3*cosd(theta3+theta1) l2*cosd(theta2+theta1)+l3*cos(theta3+theta1) l3*cosd(theta3+theta1);
    1 1 1;];
    
cq = inv(q) * pv'; %can use 1/q
   
theta1f = cq(1)+theta1;

theta2f = cq(2)+ theta2;

theta3f = cq(3)+ theta3;

end


