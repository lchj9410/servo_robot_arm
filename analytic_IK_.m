clc;clear;
syms q1 q2 x y z arm_L XYLength l1 l2;
% l1=0.19;l2=sqrt(0.225^2+0.03^2);l2_ang_offset=atan2(0.03,0.225);
% origin_offset_angle=pi/2-atan2(1,2);
% %arm_extend_length=solve(x^2+y^2-arm_L^2+(0.02^2+0.01^2)-2*sqrt(0.02^2+0.01^2)*arm_L*cos(pi-origin_offset_angle),arm_L);
% arm_extend_length=(4*x^2 + 4*y^2 + 3/1250)^(1/2)/2 + 1/100;
% angle_offset=asin(sin(pi-origin_offset_angle)/sqrt(x^2+y^2)*sqrt(0.01^2+0.02^2));
% true_angle=atan2(y,-x)+angle_offset;

solution=solve([-l1*sin(q1)+l2*cos(q1+q2)-XYLength,l1*cos(q1)+l2*sin(q1+q2)-z],[q1,q2]);
s1=solution.q1;
s2=solution.q2;
matlabFunction(s1,'file','s1');
matlabFunction(s2,'file','s2');