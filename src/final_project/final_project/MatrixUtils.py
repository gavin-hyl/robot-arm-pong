import numpy as np

def weighted_approx_pinv(A, W=None, gamma=0.1):
    if W is None:
        W = np.eye(A.shape[0])
    return (A.T @ W**2 @ A + gamma**2 * np.eye(A.shape[1])) @ A.T @ W**2