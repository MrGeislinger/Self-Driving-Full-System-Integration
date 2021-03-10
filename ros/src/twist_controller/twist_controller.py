from pid import  PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.yaw_controller = YawController(
                                    wheel_base=kwargs.get('wheel_base'), 
                                    steer_ratio=kwargs.get('steer_ratio'), 
                                    min_speed=0.1,
                                    max_lat_accel=kwargs.get('max_lat_accel'),
                                    max_steer_angle=kwargs.get('max_steer_angle')
        )

        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0 # Minimum throttle value
        mx = 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp,ki,kd,mn,mx)

        tau = 0.5 # cutoff_freq = 1/(2pi*tau)
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = kwargs.get('vehicle_mass')
        self.fuel_capacity = kwargs.get('fuel_capacity')
        self.brake_deadband = kwargs.get('brake_deadband')
        self.decel_limit = kwargs.get('decel_limit')
        self.accel_limit = kwargs.get('accel_limit')
        self.wheel_radius = kwargs.get('wheel_radius')

        self.last_time = rospy.get_time()

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = 0.
        brake = 0.
        next_steer = 0.

        if not kwargs.get('dbw_enabled'):
            self.throttle_controller.reset()
            # throttle, brake, steer
            return throttle, brake, next_steer 

        current_vel = self.vel_lpf.filt(kwargs.get('current_vel'))
        linear_vel = kwargs.get('linear_vel')
        angular_vel = kwargs.get('angular_vel')

        #

        next_steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        # Sample time must be at the same scale
        sample_time = (current_time - self.last_time) / 1e9
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel  == 0.0 and current_vel < 0.1:
            throttle = 0
            # N*m - to hold the car in place if stopped at a light (Accel - 1 m/s^2)
            brake = 400 
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel  = max(vel_error, self.decel_limit)
            # Torque in N*m; Absolute value since negative decel is positive break
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius

        rospy.loginfo('SENDING - [dbw_enabled]:[{}]'.format(kwargs.get('dbw_enabled')))
        rospy.loginfo('SENDING - [linear_vel,angular_vel,current_vel]:[{:.4f},{:.4f},{:.4f}]'.format(linear_vel, angular_vel, current_vel))
        rospy.loginfo('SENDING - [throttle,brake,steer]:[{:.4f},{:.4f},{:.4f}]'.format(throttle, brake, next_steer))


        return throttle, brake, next_steer
