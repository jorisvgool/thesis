import numpy as np
import custom_functions as cf

l = 2
c = 0.1
# Incidence matrix
B = np.array([
[1, 0, -1],
[-1, 1, 0],
[0, -1, 1]
])
B_bar = np.kron(B, np.eye(2))

# Desired edge lengths
d = np.array([np.sqrt(9), np.sqrt(9), np.sqrt(9)])

# Reference fixed points (anchors)
a = np.array([0.0,0.0])
b = np.array([2.598,1.5])

# Formation-based rotation
alpha = 0.5573
omega = 2*np.pi*10
mu = alpha*omega*np.array([1,1,1])
mu_tilde = alpha*omega*np.array([1,1,1])

# Define A matrix
A = np.zeros_like(B, dtype=float)
for i in range(3):
    for k in range(3):
        if B[i, k] == 1:
            A[i, k] = mu[k]
        elif B[i, k] == -1:
            A[i, k] = mu_tilde[k]
A_bar = np.kron(A, np.eye(2))
print(A_bar)

# Start position drone
t1 = np.array([0.0,3.0,0.0])
t2 = np.array([0.0,0.0,0.0])
t3 = np.array([-2.598,1.5,0.0])

# Absolute and relative positions
p = np.array([t1[0], t1[1], t2[0], t2[1], t3[0], t3[1]])
z = B_bar.T @ p

# Calculate control parameters
z_norm = np.linalg.norm(z.reshape((3, 2)), axis=1)
e = z_norm**l - d**l   
z_tilde = z_norm**(l-2)
Dz = cf.block_diag(z)
Dz_tilde = np.diag(z_tilde)

p_dot = -c * B_bar @ Dz @ Dz_tilde @ e + A_bar @ z
print(p_dot)
print(e)
print(A_bar @ z)