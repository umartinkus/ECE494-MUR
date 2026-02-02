import numpy as np

# Build the 6x6 matrix from your screenshot
A = np.array([
    [0.0,      0.0,    -0.707,   0.707,  -0.707,   0.707],
    [1.0,      1.0,     0.0,     0.0,     0.0,     0.0],
    [0.0,      0.0,     0.707,   0.707,   0.707,   0.707],
    [0.0,      0.0,     0.1062,  0.1095, -0.1095, -0.1062],
    [0.0,      0.0,     0.1062, -0.1036, -0.1036,  0.1062],
    [-0.135, 0.135,-0.1062,  0.1036, -0.1036,  0.1036],
], dtype=np.float64)

np.set_printoptions(precision=6, suppress=True)

detA = np.linalg.det(A)
condA = np.linalg.cond(A)

print("A =\n", A)
print("\ndet(A) =", detA)
print("cond(A) =", condA)

# Try inverse; if it's too ill-conditioned, use pseudoinverse instead
if abs(detA) < 1e-12 or condA > 1e12:
    print("\nMatrix is singular/ill-conditioned -> using pseudoinverse pinv(A).")
    A_inv = np.linalg.pinv(A)
else:
    print("\nUsing true inverse inv(A).")
    A_inv = np.linalg.inv(A)

print("\nA_inv =\n", A_inv)

# Quick verification: should be close to identity
I_check = A @ A_inv
print("\nA @ A_inv =\n", I_check)
print("\nmax |A @ A_inv - I| =", np.max(np.abs(I_check - np.eye(6))))

