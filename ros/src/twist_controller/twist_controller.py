from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

THORTTLE_CONTROLLER_KP = 1
THORTTLE_CONTROLLER_KI = 0
THORTTLE_CONTROLLER_KD = 0.05

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
	self.min_speed = 0
	self.yaw_controller = YawController(kwargs['wheel_base'], 
					    kwargs['steer_ratio'], 
					    self.min_speed, 
					    kwargs['max_lat_accel'], 
					    kwargs['max_steer_angle'])

	self.accel_limit = kwargs['accel_limit']
	self.decel_limit = kwargs['decel_limit']
	self.vehicle_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity'] * GAS_DENSITY
	self.fuel_capacity = kwargs['fuel_capacity']
	self.brake_deadband = kwargs['brake_deadband']
	self.wheel_radius = kwargs['wheel_radius']

	self.throttle_pid_controller = PID(THORTTLE_CONTROLLER_KP, 
					   THORTTLE_CONTROLLER_KI, 
					   THORTTLE_CONTROLLER_KD, 
					   mn=self.decel_limit, 
					   mx=self.accel_limit)
	

    def control(self, current_velocity, twist_cmd, dbw_enabled, elapsed_time):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

	throttle =0.0
	brake=0.0
	steer=0.0

	if not all((current_velocity, twist_cmd)):
	    return throttle, brake, steer

	velocity_error = twist_cmd.twist.linear.x - current_velocity.twist.linear.x

	throttle = self.throttle_pid_controller.step(velocity_error, elapsed_time)
	throttle = max(0.0, min(1.0, throttle))

	if velocity_error < 0:
	    throttle =0.0
	    brake = self.vehicle_mass * -velocity_error/elapsed_time * self.wheel_radius
	    if brake < self.brake_deadband:
		brake = self.brake_deadband

        steer = self.yaw_controller.get_steering(twist_cmd.twist.linear.x,
						 twist_cmd.twist.angular.z,
						 current_velocity.twist.linear.x)

	return throttle, brake, steer

