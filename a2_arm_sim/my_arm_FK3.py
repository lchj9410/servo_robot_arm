import numpy as np
from math import *
def FK3(q0,q1,q2):
	t2 = cos(q0) 
	t3 = cos(q1) 
	t4 = cos(q2) 
	t5 = sin(q0) 
	t6 = sin(q1) 
	t7 = sin(q2) 
	end_pos3 = np.array([t2*(3.0/1.25e+2)-t5*(4.7e+1/2.0e+2), t2*(3.0/2.0e+2)+t5/5.0e+1-t6*(1.9e+1/1.0e+2)+(t3*t5)/2.5e+2-(t3*t7)/5.0e+1-(t4*t6)/5.0e+1+t2*(t3*t4-t6*t7)*(1.1e+1/5.0e+1), t3*(1.9e+1/1.0e+2)+(t3*t4)/5.0e+1+(t5*t6)/2.5e+2-(t6*t7)/5.0e+1+t2*(t3*t7+t4*t6)*(1.1e+1/5.0e+1)+9.0/2.0e+2])
	return end_pos3