function s2 = s2(XYLength,l1,l2,z)
%S2
%    S2 = S2(XYLENGTH,L1,L2,Z)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    20-Mar-2021 23:38:54

t2 = XYLength.^2;
t3 = l1.^2;
t4 = l2.^2;
t5 = z.^2;
t6 = l1.*l2.*2.0;
t7 = -t2;
t8 = -t3;
t9 = -t4;
t10 = -t5;
t11 = t2+t5+t8+t9;
t13 = t3+t4+t6+t7+t10;
t12 = 1.0./t11;
t14 = t6+t11;
t15 = t13.*t14;
t16 = sqrt(t15);
s2 = [atan(t12.*(t6+t16)).*2.0;atan(t12.*(t6-t16)).*2.0];
