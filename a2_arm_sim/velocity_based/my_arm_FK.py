import numpy as np
from math import *
def FK(elbow,shoulder,turn,wristx,wristy,wristz):
	t2 = sin(shoulder) 
	t3 = sin(turn) 
	t4 = sin(elbow) 
	t5 = cos(elbow) 
	t6 = cos(shoulder) 
	t7 = cos(wristx) 
	t8 = t2*t3*t5 
	t9 = t3*t4*t6 
	t10 = t8+t9 
	t11 = sin(wristx) 
	t12 = t2*t3*t4 
	t13 = t12-t3*t5*t6 
	t14 = cos(turn) 
	t15 = sin(wristy) 
	t16 = cos(wristy) 
	t17 = t2*t5*t14 
	t18 = t4*t6*t14 
	t19 = t17+t18 
	t20 = t2*t4*t14 
	t21 = t20-t5*t6*t14 
	t22 = t2*t4 
	t23 = t22-t5*t6 
	t24 = t2*t5 
	t25 = t4*t6 
	t26 = t24+t25 
	fk =np.array( [t2*t3*(3.0/1.0e1)+t7*t10*(1.0/2.0e1)-t11*t13*(1.0/2.0e1)+t14*t15*(1.0/1.0e1)+t16*(t7*t10-t11*t13)*(1.0/1.0e1)+t2*t3*t5*(7.0/2.0e1)+t3*t4*t6*(7.0/2.0e1), t2*t14*(-3.0/1.0e1)+t3*t15*(1.0/1.0e1)-t7*t19*(1.0/2.0e1)+t11*t21*(1.0/2.0e1)-t16*(t7*t19-t11*t21)*(1.0/1.0e1)-t2*t5*t14*(7.0/2.0e1)-t4*t6*t14*(7.0/2.0e1), t6*(3.0/1.0e1)-t2*t4*(7.0/2.0e1)+t5*t6*(7.0/2.0e1)-t7*t23*(1.0/2.0e1)-t11*t26*(1.0/2.0e1)-t16*(t7*t23+t11*t26)*(1.0/1.0e1)+1.0/5.0] )
	return fk