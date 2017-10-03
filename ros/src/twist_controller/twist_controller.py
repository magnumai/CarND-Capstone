from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base, #YawController
                 steer_ratio, #YawController
                 max_steer_angle): #YawController

        kp = 0.2
        ki = 0.0009
        kd = 1.5
        
        self.pid = PID(kp, ki, kd)

        # # Find right values
        # tau = 1.0
        # ts = 1.0
        # self.low_pass_filter = LowPassFilter(tau, ts)

        # # Find right values
        # min_speed = 0.0        
        # max_lat_accel = accel_limit

        # self.yaw_controller = YawController(wheel_base,
        #                                     steer_ratio,
        #                                     min_speed,
        #                                     max_lat_accel,
        #                                     max_steer_angle)

    def control(self,
                dbw_enabled,
                cte_error,
                sample_time):

        throttle = 1.0
        brake = 0.0
        steer = 0.0
        
        if not dbw_enabled:
            self.pid.reset()
        else:
            steer = self.pid.step(cte_error, sample_time)
        
        return throttle, brake, steer
