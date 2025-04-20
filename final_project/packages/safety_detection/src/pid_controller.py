import math

# PID controller variables
simple_pid = {
    "kp": -0.025, #-0.025
    "ki": 0,
    "kd": -0.0125, #-0.0125
    "previous_error": 0,
    "integral": 0
}

# PID controller variables for yellow and white lines following
yellow_white_pid = {
    "kp": -0.025, #-0.025
    "ki": 0,
    "kd": -0.001, #-0.0125
    "previous_error": 0,
    "integral": 0
}

# bot pid values
bot_following_pid = {
    "kp": 0.000023, #[0.00001, 0.000025]
    "ki": 0,
    "kd": 0,
    "previous_error": 0,
    "integral": 0
}

# PID values for parking
parking_pid = { 
    "kp": -0.035,
    "ki": 0,
    "kd": 0,
    "previous_error": 0,
    "integral": 0
}

def pid_controller(pid, error, dt, reset=False):
    '''
    The method to get PID controls.
    For P/PD, just set ki and/or kd to 0
    use the reset flag when the desired value changes a lot
    need to tune the kp, ki, kd values for different tasks (keep a note of them)
    '''
    if reset:
        pid['integral'] = 0
        pid['previous_error'] = 0
    # error = desired_value - measured_value
    pid['integral'] += error * dt
    derivative = (error - pid['previous_error']) / dt if dt > 0 else 0
    
    output = (pid['kp'] * error) + (pid['ki'] * pid['integral']) + (pid['kd'] * derivative)
    pid['previous_error'] = error
    
    return output
    
def pid_controller_v_omega(error, pid_values, rate, reset=False):
    dt = 1 / rate
    # feed the error into the pid function to get the amount to turn the bot
    # also do clamping
    omega = None
    if error is not None:
        omega = pid_controller(pid_values, error, dt, reset=reset)
        clamp_value = (math.pi) * 1
        #omega = max(-clamp_value, min(omega, clamp_value))
    # return the calculated v and omega
    if error is None:
        return 0, 0
    else:
        return 0.24, omega

def bot_and_lane_controller(lane_error, bot_error, lane_pid, bot_pid, rate, reset=False):
    dt = 1 / rate
    # feed the error into the pid function to get the amount to turn the bot
    # also do clamping
    omega = None
    if lane_error is not None:
        omega = pid_controller(lane_pid, lane_error, dt, reset=reset)
        clamp_value = (math.pi) * 1
        #omega = max(-clamp_value, min(omega, clamp_value))
    v = 0.23
    if bot_error is not None:
        v = pid_controller(bot_pid, bot_error, dt, reset=reset)
        max_v = 0.23
        min_v = 0
        v = max(min_v, min(v, max_v))

    # set velocity to 0 if lane is not seen
    if omega is None: v = 0

    # return the calculated v and omega
    return v, omega
