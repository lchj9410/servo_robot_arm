import numpy as np
from math import *
def jacobian3(q0,q1,q2):
	t2 = cos(q0) 
	t3 = cos(q1) 
	t4 = cos(q2) 
	t5 = sin(q0) 
	t6 = sin(q1) 
	t7 = sin(q2) 
	t8 = t3*t4 
	t9 = t3*t7 
	t10 = t4*t6 
	t11 = t6*t7 
	t12 = -t11 
	t13 = t8/5.0e+1 
	t14 = t9/5.0e+1 
	t15 = t10/5.0e+1 
	t16 = t11/5.0e+1 
	t20 = t9+t10 
	t17 = -t13 
	t18 = -t14 
	t19 = -t15 
	t21 = t8+t12 
	t22 = t2*t20*(1.1e+1/5.0e+1) 
	t23 = -t22 
	t24 = t2*t21*(1.1e+1/5.0e+1) 
	J_pos = np.reshape([t2*(-4.7e+1/2.0e+2)-t5*(3.0/1.25e+2),t2/5.0e+1-t5*(3.0/2.0e+2)+(t2*t3)/2.5e+2-t5*t21*(1.1e+1/5.0e+1),(t2*t6)/2.5e+2-t5*t20*(1.1e+1/5.0e+1),0.0,t3*(-1.9e+1/1.0e+2)+t16+t17+t23-(t5*t6)/2.5e+2,t6*(-1.9e+1/1.0e+2)+t18+t19+t24+(t3*t5)/2.5e+2,0.0,t16+t17+t23,t18+t19+t24],[3,3]) 


	return J_pos