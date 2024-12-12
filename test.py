import numpy as np
from math import fmod

def wrap_q(q):
    q_wrapped = q.copy()
    for i, qi in enumerate(q):
        q_wrapped[i] = fmod(qi + np.pi, 2*np.pi) - np.pi
    return q_wrapped

q = np.array([0, 2 * np.pi, np.pi * 4/3, 1/3 * np.pi])
print(wrap_q(q) / np.pi)
