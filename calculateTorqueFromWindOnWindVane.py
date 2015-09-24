#Parameters to change for simulation
#This assumes a rectangular, thin-plate is used as the wind vane
v = 1.5 #velocity of the wind in mph
h = 1 #height of the windvane in inches
L = 3 #length of the windvane in inches from the center pivot point


#Parameters that shouldn't be changed under normal use: 
Cd = 1.28 #unitless coefficient of drag for a rectangular thin plate
#from: https://www.grc.nasa.gov/www/K-12/airplane/shaped.html

#Convert units:
def in_to_ft(length):
	return length/12.0

#Convert length and height of the windvane to ft for equation
h = in_to_ft(h) #ft
L = in_to_ft(L) #ft


#calculate the wind pressure:
Pw = .00256*(v**2) #psf (lbf/ft^2)
#from: https://www.arraysolutions.com/Products/windloads.htm


#calculate the torque experienced by a  wind vane from wind 
#blowing over it.
# Derivation:
#  F = A*Pw*Cd
#Since this acts over the length of the plate:
#  torque = Pw * Cd *  * h * integral of l from 0 to L.
#This reduces to:
torque = Pw * 1.28 * h * (L**2/2)

print 'torque is: ', torque, ' lbf * ft'