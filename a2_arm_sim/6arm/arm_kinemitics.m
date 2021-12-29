clc;clear;
syms q0 q1 q2 ;% q3 q4 q5;
q=[q0 q1 q2];
R0=rz(q0);R1=rx(q1);R2=rx(q2);%R3=ry(q3);R4=rz(-q4);R5=rx(q5);
end_pos3=[0;0;0.045]+R0*[0.02;0.015;0]+R1*R0*[0.004;0;0.19]+R2*R1*R0*[0;0.22;0.02];

vpa(subs(end_pos3,q,[0 0 0]))
J_pos=jacobian(end_pos3,q);


matlabFunction(J_pos,'file','J_pos');
matlabFunction(end_pos3,'file','end_pos3');
function R=rx(ang)
R=[1 0 0;0 cos(ang) -sin(ang);0 sin(ang) cos(ang)];
end
function R=ry(p)
R=[cos(p) 0 sin(p);0 1 0;-sin(p) 0 cos(p)];
end
function R=rz(y)
R=[cos(y) -sin(y) 0;sin(y) cos(y) 0;0 0 1];
end