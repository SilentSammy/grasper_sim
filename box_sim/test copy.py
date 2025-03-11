import numpy as np

# Define the restricted actuator matrix, A_restricted (6x7):
A_restricted = np.array([
    [ 1,  0,  1,  0,  1,  0,  0],   # Contribution to Fx
    [ 0,  1,  0,  1,  0,  1,  0],   # Contribution to Fy
    [ 0,  0,  0,  0,  0,  0,  1],   # Contribution to Fz (only bottom thruster)
    [ 0, -2,  0, -2,  0,  2,  0],   # Contribution to Tx
    [ 2,  0,  2,  0, -2,  0,  0],   # Contribution to Ty
    [-5,  0,  5,  0,  0,  0,  0]    # Contribution to Tz
])

print("Restricted Actuator Matrix A_restricted:")
print(A_restricted)
print("Shape of A_restricted:", A_restricted.shape)

# Compute the pseudoinverse of A_restricted.
# The pseudoinverse will be a (7 x 6) matrix.
A_restricted_pinv = np.linalg.pinv(A_restricted)
print("\nPseudoinverse of A_restricted (A_restricted_pinv):")
print(A_restricted_pinv)
print("Shape of A_restricted_pinv:", A_restricted_pinv.shape)

# Convert the pseudoinverse matrix to a list of lists with regular floats
A_restricted_pinv_list = A_restricted_pinv.tolist()

# For clarity, format the output as a Python list-of-lists so that you can copy it
def format_matrix(mat):
    formatted = "[\n"
    for row in mat:
        formatted += "    " + str(row) + ",\n"
    formatted += "]"
    return formatted

print("\nCopy this as your PRECOMPUTED_RESTRICTED_A_PINV:")
print(format_matrix(A_restricted_pinv_list))