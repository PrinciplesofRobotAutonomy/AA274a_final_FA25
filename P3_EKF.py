import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

dt = 0.05

### Dynamics and Measurement Equations ###

def dynamics_model(
    X: np.ndarray, 
    v_e: float, 
    omega_e: float,
    v_r: float,
    omega_r: float
) -> np.ndarray:
    """Kalman Filter dynamics model.

    Args:
        X (np.ndarray): Combined state of ego and target robots.
        v_e (float): Linear velocity command (ego).
        omega_e (float): Angular velocity command (ego).
        v_r (float): Linear velocity command (target).
        omega_r (float): Angular velocity command (target).

    Returns:
        np.ndarray: Next state of robots.
    """
    return np.reshape(np.array([X[0] + dt * v_e * np.cos(X[2]),
                     X[1] + dt * v_e * np.sin(X[2]),
                     X[2] + dt * omega_e,
                     X[3] + dt * v_r * np.cos(X[5]),
                     X[4] + dt * v_r * np.sin(X[5]),
                     X[5] + dt * omega_r]),(6,1))


def measurement_model(
    X: np.ndarray
) -> np.ndarray:
    """Kalman Filter measurement model.

    Args:
        X (np.ndarray): Combined state of ego and target robots.
    
    Returns:
        np.ndarray: Expected measurement.
    """
    x_e = X[0]
    y_e = X[1]
    th_e = X[2]
    
    x_r = X[3]
    y_r = X[4]
    
    rot_e = np.array([[np.cos(th_e), -np.sin(th_e)],
                        [np.sin(th_e), np.cos(th_e)]])
    pdiff_re = np.array([[x_r - x_e], [y_r - y_e]])

    return rot_e.T @ pdiff_re


def getG(
    X: np.ndarray, 
    v_e: float, 
    omega_e: float,
    v_r: float,
    omega_r: float
):
    """Computes the Jacobian of the dynamics model with respect 
    to the robots' state and input commands.
    
    Args:
        X (np.ndarray): Robots' state.
        v_e (float): Linear velocity command (ego).
        omega_e (float): Angular velocity command (ego).
        v_r (float): Linear velocity command (target).
        omega_r (float): Angular velocity command (target).
    
    Returns:
        G (np.ndarray): Jacobian of the dynamics model.
    """
    ####################### Code starts here #######################





    
    ####################### Code ends here #######################
    return G


def getH(
    X: np.ndarray
) -> np.ndarray:
    """Computes the Jacobian of the measurement model with respect
    to the robots' state.
    
    Args:
        X (np.ndarray): Robots' state.
    
    Returns:
        H (np.ndarray): Jacobian of the measurement model.
    """
    ####################### Code starts here #######################





    
    ####################### Code ends here #######################
    return H

### Navigation Relative to Track ###

def line_following_command(
    x: float,
    y: float,
    theta: float,
    line_matrix: np.ndarray) -> (float,float):
    """Computes the control action for the vehicle to follow the track
    
    Args:
        x (float): Robot position in x axis.
        y (float): Robot position in y axis.
        theta (float): Robot heading at current time step.
        line_matrix (np.ndarray): Matrix of 2D positions describing the line to follow
    
    Returns:
        (v, omega) (float,float): Commanded longitudinal velocity and angular velocity
    """

    N = np.shape(line_matrix)[1] #number of positions in the matrix describing the line
    
    # 1. Identify the index position of the closest point in the line to the input position
    idx = np.argmin(np.linalg.norm(line_matrix - np.array([[x],[y]]),axis=0))
    
    ####################### Code starts here #######################
    # 2. Find the positional values of the next point in the line (Hint: use the % operator to loop back around if you reach the end)
    
    # 3. Use np.arctan2() and the difference between this and the current position to determine the heading necessary to face the next point
    
    # 4. Calculate the omega value necessary to match this heading at the next time step
    # HINT: the magnitude of this value depends on the time step size, "dt"
    
    # 5. Set the linear velocity at which the vehicle should move
    
    ####################### Code ends here #######################
    return (v, omega)

### Plot Results ###

def plot_error_ellipse(ax, mean, cov, colorEll='red'):
    # Calculate the error ellipse parameters
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    order = eigenvalues.argsort()[::-1]
    eigenvalues, eigenvectors = eigenvalues[order], eigenvectors[:, order]
    angle = np.degrees(np.arctan2(*eigenvectors[:,0][::-1]))
    
    # Compute the radius of the ellipse to correspond to the desired confidence level
    chi2_val = 2.4477  # Corresponds to 95% conf. interval
    width, height = 2 * chi2_val * np.sqrt(eigenvalues)

    # Draw the ellipse
    ellipse = patches.Ellipse(mean, width, height, angle=angle, edgecolor=colorEll, fc='None', lw=2)
    ax.add_patch(ellipse)