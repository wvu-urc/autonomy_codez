import time 

class PidController():
	def __init__(self):
		self.previous_time = time.time()
		self.previous_error = 0
		self.accumuated_error = 0
		self.val = 0

	def update(self, current_error: float, kp:float, ki: float, kd:float, total_gain: float) -> float:
		'''Runs an interation of the control loop and returns the output control value'''
		curr_time = time.time()
		self.accumuated_error += self.previous_error

		result_p = kp * current_error
		result_d = kd * ((current_error - self.previous_error) / (curr_time - self.previous_time))
		result_i = ki * self.accumuated_error
		self.val = (result_p + result_d + result_i) * total_gain
		self.previous_error = current_error
		self.previous_time = curr_time

		return self.val
	
	def reset_dt(self):
		'''makes the previous time right now'''
		self.previous_time = time.time()

	def reset_error(self):
		'''makes the previous error zero'''
		self.previous_error = 0

	def reset_error_integration(self):
		'''makes the accumulated error zero'''
		self.accumuated_error = 0

	def reset_controller(self,):
		'''resets the previous time, previous error, and curren accumulated error'''
		self.reset_dt()
		self.reset_error()
		self.reset_error_integration()