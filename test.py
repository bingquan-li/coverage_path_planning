from itertools import product

import numpy as np

L_SAMPLES = np.arange(-3.0, 3.01, 1.0)
S_STEP = 6.0
S_SAMPLES = np.arange(0.0, 30.0 + S_STEP / 2, S_STEP)

s = product(L_SAMPLES, repeat=len(S_SAMPLES) - 1)

print(list(s))