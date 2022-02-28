import sys

print('Usage: python calca3a4.py p_0, p_f, pdot_0, pdot_f, tf')

p_0     = float(sys.argv[1])
p_f     = float(sys.argv[2])
pdot_0  = float(sys.argv[3])
pdot_f  = float(sys.argv[4])
tf      = float(sys.argv[5])

a3 = (3*p_f - 3*p_0 - 2*pdot_0*tf - pdot_f*tf)/tf**2
a4 = (2*p_0 + (pdot_0+pdot_f)*tf - 2*p_f)/tf**3

print('a1: ',p_0)
print('a2: ',pdot_0)
print('a3: ',a3)
print('a4: ',a4)
