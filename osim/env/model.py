import opensim
import time
import numpy as np

integrator_accuracy = 3e-2
stepsize = 0.01
model = opensim.Model('../models/gait14dof22musc_pros_20180507.osim')

# initialise model's simbody System object. System is a computational representation of the Model
model.initSystem()
# initialise the ModelVisualizer object
model.setUseVisualizer(True)

'''
Defines controller for controlling actuators. A controller computes and sets the values of the controls for the actuators under its control
PrescribedController specifies functions that prescribe the control values of its actuators as a function of time. 
So we will explicitly provide these functions.
'''
brain = opensim.PrescribedController()

muscleSet = model.getMuscles()
for j in range(muscleSet.getSize()):
	func = opensim.Constant(1.0)
	brain.addActuator(muscleSet.get(j))
	brain.prescribeControlForActuator(j, func)

model.addController(brain)
model.initSystem()

########### System initialised
# time.sleep(3)

initial_state = model.initializeState()
initial_state.setTime(0)
istep = 0
manager = opensim.Manager(model)
def reset_manager():
	manager.setIntegratorAccuracy(integrator_accuracy)
	manager.initialize(initial_state)

def reset():
	istep = 0
	reset_manager()

reset()

def integrate():
	global istep
	# Define the new endtime of the simulation
	istep = istep + 1

	# Integrate till the new endtime
	try:
		state = manager.integrate(stepsize * istep)
	except Exception as e:
		print (e)

def actuate(action):
	if np.any(np.isnan(action)):
		raise ValueError("NaN passed in the activation vector. Values in [0,1] interval are required.")

	brain = opensim.PrescribedController.safeDownCast(model.getControllerSet().get(0))
	functionSet = brain.get_ControlFunctions()

	for j in range(functionSet.getSize()):
		func = opensim.Constant.safeDownCast(functionSet.get(j))
		func.setValue( float(action[j]) )

action = [1.0] * muscleSet.getSize()
# actuate(action)
for i in range(100):
	integrate()
time.sleep(3)
