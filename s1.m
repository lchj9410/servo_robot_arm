function s1 = s1(XYLength,l1,l2,z)
%S1
%    S1 = S1(XYLENGTH,L1,L2,Z)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    20-Mar-2021 23:38:54

t2 = XYLength.^2;
t3 = l1.^2;
t4 = l2.^2;
t5 = z.^2;
t6 = XYLength.*l1.*2.0;
t7 = l1.*l2.*2.0;
t8 = l1.*z.*2.0;
t9 = -t7;
t10 = -t2;
t11 = -t3;
t12 = -t4;
t13 = -t5;
t14 = t2+t5+t11+t12;
t15 = t2+t3+t5+t8+t12;
t17 = t3+t4+t7+t10+t13;
t16 = 1.0./t14;
t18 = t7+t14;
t19 = 1.0./t15;
t20 = t17.*t18;
t21 = sqrt(t20);
t22 = -t21;
t23 = t7+t21;
t24 = t7+t22;
s1 = [atan(t19.*(t6+t9+t2.*t16.*t23+t5.*t16.*t23+t11.*t16.*t23+t12.*t16.*t23)).*-2.0;atan(t19.*(t6+t9+t2.*t16.*t24+t5.*t16.*t24+t11.*t16.*t24+t12.*t16.*t24)).*-2.0];