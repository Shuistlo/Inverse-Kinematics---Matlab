function [theta1f,theta2f,theta3f]=IK_Jacobian_func(px,py,pxk,pyk,theta_total,theta_totalk,theta1,theta2,theta3,l1,l2,l3)
%pxk, pyk, theta_totalK are the curret position and orientation        
pv = [px-pxk py-pyk theta_total-theta_totalk];

((abs(pv(1))>0.0001)&&(abs(pv(2))>0.0001))
    




end


