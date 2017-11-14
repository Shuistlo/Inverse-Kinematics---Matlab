function [theta1, theta2, theta3]=IK_analytical_func(px,py,theta_total,l1,l2,l3)
%have to do theta2 first
pwx = px - (l3*cos(theta_total));
pwy = py - (l3*sin(theta_total));

c2 = (pwx^2 + pwy^2 - l1^2 -l2^2)/(2*l1*l2);
s2 = sqrt(1- c2^2);

theta2 = atan2(s2, c2);

s1 = ((l1+ (l2*c2))*pwy - (l2*s2*pwx))/(pwx^2 + pwy^2);
c1 = ((l1 + l2*c2)*pwx + (l2*s2*pwy))/(pwx^2 + pwy^2);

theta1 = atan2(s1, c1);

theta3 = theta_total - theta1 - theta2;

end
