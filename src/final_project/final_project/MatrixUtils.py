import numpy as np

def weighted_pinv(A, gamma=0.1):
    return np.linalg.pinv(A.T @ A + gamma**2 * np.eye(A.shape[1])) @ A.T