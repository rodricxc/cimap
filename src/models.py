import numpy as np


def sample_motion_model_odometry(u_t, x_t1, error, type_distrib='normal'):
    """
    Sampling poses x_t = [_x, _y, _theta] given the last state 'x_t1', the transition probability
    and the last two odometry results obtained as 'u_t'.
    Book: Probabilistic Robotics, page 136, Table 5.6
    
    :param u_t: Control input based on odometry. The control is a differentiable set of two pose estimates obtained by the robots odometrer. 
            Represented as [xb0 xb1], with 'xb_0' the odometry in the last state, and 'xb_1' the odometry in 
            the current state. The given 'u_t' is represented as [[x_b, y_b, theta_b], [x_bl, y_bl, theta_bl]]
    :param x_t1: Last state position. Represented as [x, y, theta] 
    :param error: transaction(a_1, a_2), angular(a_3, a_4) and rotational(a_5, a_6) error for the 
            model defined as [a_1, a_2, a_3, a_4, a_5, a_6]
    :param type_distrib: 'normal' | 'triangular' type of distribution applied

    :return: x_t = [_x, _y, _theta]  
    """
    xb_0, xb_1 = u_t
    x_b, y_b, theta_b = xb_0
    x_bl, y_bl, theta_bl = xb_1
    x, y, theta = x_t1
    
    delta_rot1 = np.arctan2(y_bl - y_b, x_bl - x_b) - theta_b
    delta_trans = ((x_b - x_bl) ** 2 + (y_b - y_bl) ** 2)**0.5
    delta_rot2 = theta_bl - theta_b - delta_rot1

    delta_ch_rot1 = delta_rot1   - sample(error[0] * abs(delta_rot1) + error[1] * delta_trans, type=type_distrib)
    delta_ch_trans = delta_trans - sample(error[2] * delta_trans + error[3] * (abs(delta_rot1) + abs(delta_rot2)), type=type_distrib)
    delta_ch_rot2 = delta_rot2   - sample(error[0] * abs(delta_rot2) + error[1] * delta_trans, type=type_distrib)

    _x = x + delta_ch_trans * np.cos(theta + delta_ch_rot1)
    _y = y + delta_ch_trans * np.sin(theta + delta_ch_rot1)
    _theta = theta + delta_ch_rot1 + delta_ch_rot2

    x_t = [_x, _y, _theta]
    return x_t

def motion_model_odometry_to_control(u_t, error):
    """
    Retrieve control and error from odometry 
    based on Book: Probabilistic Robotics, page 136, Table 5.6
    
    :param u_t: Control input based on odometry. The control is a differentiable set of two pose estimates obtained by the robots odometrer. 
            Represented as [xb0 xb1], with 'xb_0' the odometry in the last state, and 'xb_1' the odometry in 
            the current state. The given 'u_t' is represented as [[x_b, y_b, theta_b], [x_bl, y_bl, theta_bl]]
    :param error: transaction(a_1, a_2), angular(a_3, a_4) and rotational(a_5, a_6) error for the 
            model defined as [a_1, a_2, a_3, a_4, a_5, a_6]
    :return: u, E representiong the control vector and the error estimated cov matrix  
    """
    xb_0, xb_1 = u_t
    x_b, y_b, theta_b = xb_0
    x_bl, y_bl, theta_bl = xb_1
    
    
    delta_rot1 = np.arctan2(y_bl - y_b, x_bl - x_b) - theta_b
    delta_trans = ((x_b - x_bl) ** 2 + (y_b - y_bl) ** 2)**0.5

    if (x_bl - x_b) * np.cos(theta_b) + (y_bl - y_b) * np.sin(theta_b) < 0:
        delta_trans *= -1

    delta_rot2 = theta_bl - theta_b - delta_rot1

    u = np.array([delta_trans, theta_bl - theta_b])

    E = np.array([[error[2] * abs(delta_trans) + error[3] * (abs(delta_rot1) + abs(delta_rot2)), 0],
                  [0,  error[0] * abs(delta_rot1) + error[1] *abs(delta_trans) + error[0] * abs(delta_rot2) + error[1] * abs(delta_trans)]])
    
    return u, E

def sample(std, type='normal', size=None):
    """
    Sampling value from an aproximate normal or triangular distribution with zero 
    mean and standard deviation 'std'
            
    :param std: standart deviation
    :param type: 'normal' | 'triangular' type of distribution applied
    :return: Sampling value from an aproximate normal or triangular distribution with zero \ 
        mean and standard deviation 'std'
    """
    if type == 'normal':
        return sample_normal_distribution(std, size=size)
    else:
        return sample_triangular_distribution(std, size=size)

def sample_normal_distribution(std, size=None):
    if std == 0:
        return 0
    return np.random.normal(0, std, size=size)
    #return 0.5 * (np.random.uniform(-std, std, 12).sum())

def sample_triangular_distribution(std, size=None):
    if std == 0:
        return 0
    alpha = (6**0.5)*std
    return np.random.triangular(-alpha, 0, alpha, size=size)
