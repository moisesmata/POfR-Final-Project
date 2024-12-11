import os
import numpy as np
from scipy.linalg import lu_factor, lu_solve
os.environ['OMP_NUM_THREADS'] = '8'
os.environ['MKL_NUM_THREADS'] = '8'

def solve_linear_system_lu(A, b):
    """
    Solves the linear system A x = b using LU factorization.

    Parameters
    ----------
    A : array_like
        The coefficient matrix.
    b : array_like
        Right-hand side vector.
    num_threads : int, optional
        The number of threads for parallel BLAS/LAPACK routines. Default is 8.

    Returns
    -------
    x : ndarray
        The solution vector to the system A x = b.

    Raises
    ------
    ValueError
        If A is not square or dimensions of A and b are incompatible.
    """

    A = np.asarray(A, dtype=float)
    b = np.asarray(b, dtype=float)

    if A.ndim != 2 or A.shape[0] != A.shape[1]:
        raise ValueError("Matrix A must be square (n x n).")
    if b.ndim != 1 or b.shape[0] != A.shape[0]:
        raise ValueError("The dimension of b must match the dimension of A.")

    lu, piv = lu_factor(A)
    x = lu_solve((lu, piv), b)
    return x
