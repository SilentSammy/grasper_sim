import numpy as np

# Define actuator positions (from your forces array):
# For forces defined as:
# [position_x, position_y, position_z, 0, 0, 0]
# With box_dims = [8, 16, 4], we have:
#   forces[0]: [0, 5, box_dims[2]/2]  -> [0, 5, 2]
#   forces[1]: [0, -5, box_dims[2]/2] -> [0, -5, 2]
#   forces[2]: [0, 0, -box_dims[2]/2] -> [0, 0, -2]
r0 = np.array([0, 5, 2])
r1 = np.array([0, -5, 2])
r2 = np.array([0, 0, -2])

# Utility function to build the cross-product matrix:
def cross_matrix(r):
    return np.array([[    0, -r[2],  r[1]],
                     [ r[2],     0, -r[0]],
                     [-r[1],  r[0],     0]])

# The actuator matrix A maps the 9 individual force components 
# (f0, f1, f2, each in R^3) to the 6 net force/torque components:
#
# A is structured in two blocks:
#  - Top 3 rows:  f0 + f1 + f2 = desired_net_force
#  - Bottom 3 rows: r0 x f0 + r1 x f1 + r2 x f2 = desired_net_torque
I3 = np.eye(3)

# Build the top block: force contributions
A_top = np.hstack([I3, I3, I3])  # Shape (3,9)

# Build the bottom block: torque contributions via cross products
A_bot = np.hstack([cross_matrix(r0), cross_matrix(r1), cross_matrix(r2)])  # Shape (3,9)

# Construct full actuator matrix A (6x9)
A = np.vstack([A_top, A_bot])

print("Actuator matrix A:")
print(A)
print("\nShape of A:", A.shape)

# Compute the pseudoinverse of A (this will be a 9x6 matrix)
A_pinv = np.linalg.pinv(A)

print("\nPseudoinverse A_pinv:")
print(A_pinv)
print("\nShape of A_pinv:", A_pinv.shape)