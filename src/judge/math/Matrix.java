/*******************************************************************************
* File:  Matrix.java
* Date:  1/12/2011
* Auth:  K. Loux
* Desc:  Class for performing basic matrix math.  This is not a fully functional
*        matrix class - required functionality is added as the need arises.
*******************************************************************************/

// Declare our package
package judge.math;

/**
 * Represents a basic matrix.  This is not a fully functional matrix class, but
 * represents on the functionality required.  Additional functionality can be
 * added as the need arises.  Supports 1- and 2-dimensional matrices only.
 *
 * @author K. Loux
 */
public class Matrix
{
    // Fields
	private final int rowCount;
	private final int colCount;
	private double elements[][];

    // Non-zero is really non-nearly-zero, with nearly zero defined as:
    private static final double nearlyZero = 1.0e-6;

	// Methods
	/**
	 * Constructor for creating an empty matrix of the specified dimensions.
	 *
	 * @param rows	Number of rows
	 * @param cols	Number of columns
	 */
	public Matrix(int rows, int cols)
	{
		// Initialize required fields
		rowCount = rows;
		colCount = cols;

		// Create the elements array
		elements = new double[rowCount][colCount];
	}

	/**
	 * Constructor for creating a matrix from an existing 2D array of data.
	 *
	 * @param _elements	Values to be used to create the matrix object
	 */
	public Matrix(double _elements[][])
	{
		// Initialize required fields
		rowCount = _elements.length;
		colCount = _elements[0].length;

		// Assign the element array
		elements = _elements;
	}

	/**
	 * Returns the number of rows.
	 *
	 * @return Number of rows in the matrix
	 */
	public int GetRowCount()
	{
		return rowCount;
	}

	/**
	 * Returns the number of columns.
	 *
	 * @return Number of columns in the matrix.
	 */
	public int GetColCount()
	{
		return colCount;
	}

	/**
	 * Assigns the specified value to the specified location in the matrix.
	 *
	 * @param row	Index of the desired element's row (zero based)
	 * @param col	Index of the desired element's column (zero based)
	 * @param value	Value to assign to the element
	 */
	public void SetElement(int row, int col, double value)
	{
		elements[row][col] = value;
	}

	/**
	 * Returns the value of the element at the specified location.
	 *
	 * @param row	Index of the desired element's row (zero based)
	 * @param col	Index of the desired element's column (zero based)
	 *
	 * @return Value of the elements at the specified location
	 */
	public double GetElement(int row, int col)
	{
		return elements[row][col];
	}

	/**
	 * Returns the result of standard matrix multiplication.  Throws an
	 * exception if the matrices inner dimensions do not match.
	 *
	 * @param target	Matrix to be multiplied (the matrix on the right)
	 *
	 * @return a matrix representing the result of the multiplication
	 *
	 * @throws IllegalArgumentException
	 */
	public Matrix Multiply(Matrix target) throws IllegalArgumentException
	{
		// Create the result matrix
		Matrix result = new Matrix(rowCount, target.colCount);

		// Make sure this is a valid multiplication
		if (colCount != target.rowCount)
            throw new IllegalArgumentException(
					"Error:  Matrix inner dimensions do not match!");

		// Perform the multiplication
		int row, col, i;
		for (row = 0; row < result.rowCount; row++)
		{
			for (col = 0; col < result.colCount; col++)
			{
				for (i = 0; i < colCount; i++)
					result.elements[row][col] +=
							elements[row][i] * target.elements[i][col];
			}
		}

		return result;
	}
	
	/**
	 * Multiplication of a matrix with a scalar
	 * 
	 * @param target
	 * @return 
	 */
	public Matrix Multiply(double target)
	{
		// Create the result matrix
		Matrix result = new Matrix(rowCount, colCount);

		// Perform the multiplication
		int row, col;
		for (row = 0; row < result.rowCount; row++)
		{
			for (col = 0; col < result.colCount; col++)
			{
				result.elements[row][col] = elements[row][col] * target;
			}
		}

		return result;
	}
    
    /**
	 * Returns the result of standard matrix addition.  Throws an
	 * exception if the matrices dimensions do not match.
	 *
	 * @param target	Matrix to be added
	 *
	 * @return a matrix representing the result of the addition
	 *
	 * @throws IllegalArgumentException
	 */
	public Matrix Add(Matrix target) throws IllegalArgumentException
	{
		// Create the result matrix
		Matrix result = new Matrix(rowCount, colCount);

		// Make sure this is a valid operation
		if (colCount != target.colCount || rowCount != target.rowCount)
            throw new IllegalArgumentException(
					"Error:  Matrix dimensions do not match!");

		// Perform the addition
		int row, col, i;
		for (row = 0; row < result.rowCount; row++)
		{
			for (col = 0; col < result.colCount; col++)
			{
				result.elements[row][col] +=
						elements[row][col] + target.elements[row][col];
			}
		}

		return result;
	}
    
    /**
	 * Returns the result of standard matrix subtraction.  Throws an
	 * exception if the matrices dimensions do not match.
	 *
	 * @param target	Matrix to be subtracted
	 *
	 * @return a matrix representing the result of the subtraction
	 *
	 * @throws IllegalArgumentException
	 */
	public Matrix Subtract(Matrix target) throws IllegalArgumentException
	{
		// Create the result matrix
		Matrix result = new Matrix(rowCount, colCount);

		// Make sure this is a valid operation
		if (colCount != target.colCount || rowCount != target.rowCount)
            throw new IllegalArgumentException(
					"Error:  Matrix dimensions do not match!");

		// Perform the subtraction
		int row, col, i;
		for (row = 0; row < result.rowCount; row++)
		{
			for (col = 0; col < result.colCount; col++)
			{
				result.elements[row][col] +=
						elements[row][col] - target.elements[row][col];
			}
		}

		return result;
	}

	/**
	 * Returns the row rank of the matrix (number of linearly independent rows).
	 *
	 * @return Row rank of the matrix
	 */
	public int GetRank()
	{
        // Perform Gauss-Jordan elimination to produce the row-echelon form
        // The number of non-zero rows is the rank of the matrix

        // Make a copy of this object so we don't change its contents whille
        // performing this calculation
        int curCol, curRow;
        Matrix copy = new Matrix(rowCount, colCount);
        for (curRow = 0; curRow < rowCount; curRow++)
        {
            for (curCol = 0; curCol < colCount; curCol++)
                copy.elements[curRow][curCol] = elements[curRow][curCol];
        }

        // Perform Gauss-Jordan elimination
        double factor;
        int pivotRow, pivotCol = 0;
        for (pivotRow = 0; pivotRow < Math.min(rowCount, colCount); pivotRow++)
        {
            // Make sure the pivot is non-zero
            // If it is zero, move the row to the bottom and start over
            // (or if it is all zeros below, then advance to the next column)
            if (Math.abs(copy.elements[pivotRow][pivotCol]) > nearlyZero)
            {
                for (curRow = pivotRow + 1; curRow < rowCount; curRow++)
                {
                    // Scale the pivot row and add it to this row such that the
                    // element of this row in the pivot column becomes zero
					if (Math.abs(copy.elements[curRow][pivotCol]) > nearlyZero)
					{
						factor = copy.elements[pivotRow][pivotCol]
                                / copy.elements[curRow][pivotCol];

						for (curCol = pivotCol; curCol < colCount; curCol++)
							copy.elements[curRow][curCol] =
                                    copy.elements[curRow][curCol] * factor -
									copy.elements[pivotRow][curCol];
					}
                }
            }
            else
            {
				// Find a non-zero row to swap with
				for (curRow = pivotRow + 1; curRow < rowCount; curRow++)
				{
					if (Math.abs(copy.elements[curRow][pivotCol]) >
							nearlyZero)
					{
						double temp;
						for (curCol = pivotCol; curCol < colCount; curCol++)
						{
							temp = copy.elements[pivotRow][curCol];
							copy.elements[pivotRow][curCol] =
									copy.elements[curRow][curCol];
							copy.elements[curRow][curCol] = temp;
						}

						// Decrement the pivot column because we need it to
						// be the pivot column again
						pivotCol--;

						// We did the swap, so we can stop searching
						break;
					}
				}

				// If we didn't find anything, we just move on, but we
				// decrement the pivot row because we need it to be the
				// pivot row again
				pivotRow--;
            }

			// Increment the pivot column
			pivotCol++;

			// If we get all the way to the end of the matrix and don't find
			// anything, then we're done!
			if (pivotCol >= colCount)
				break;

			// Print the matrix after each loop for debugging purposes
            /*for (curRow = 0; curRow < rowCount; curRow++)
            {
                for (curCol = 0; curCol < colCount; curCol++)
                {
                    System.out.print(copy.elements[curRow][curCol] + "  ");
                }
                System.out.print("\n");
            }
            System.out.print("\n");*/
        }

        // Count the number of non-zero rows
        int rank = 0;
        for (curRow = 0; curRow < rowCount; curRow++)
        {
            for (curCol = 0; curCol < colCount; curCol++)
            {
                if (Math.abs(copy.elements[curRow][curCol]) > nearlyZero)
                {
                    // Row contained a non-zero element - increment the rank
                    // and stop looking at the other elements in this row
                    rank++;
                    break;
                }
            }
        }

		return rank;
	}

	/**
	 *  Returns the psuedo-inverse of the matrix.  We use psuedo-inverse because
	 * we want to handle rectangular matrices - this means that there is more
	 * than one combination of wheel speeds to achieve the same bot speed vector
	 * when using more than three wheels!
	 *
	 * @return The psuedo-inverse of the matrix
	 */
    public Matrix GetPsuedoinverse()
    {
        return Psuedoinverse(this);
    }

	/**
	 * Returns the psuedo-inverse of the specified matrix.  Static version of
	 * GetPsuedoinverse().  Uses Singular Value Decomposition method to calculate
	 * the psuedo-inverse.
	 *
	 * @param target	Matrix whose pseudoinverse is desired
	 *
	 * @return Psuedoinverse of the specified matrix
	 */
    static public Matrix Psuedoinverse(final Matrix target)
    {
        // Use singular value decomposition to compute the inverse
        // SVD algorithm interpreted from Numerical Recipies in C
        Matrix U = new Matrix(target.rowCount, target.colCount);
        Matrix W = new Matrix(target.colCount, target.colCount);
        Matrix V = new Matrix(target.colCount, target.colCount);

        // Copy target to U
        int i, j;
        for (i = 0; i < U.rowCount; i++)
        {
            for (j = 0; j < U.colCount; j++)
                U.elements[i][j] = target.elements[i][j];
        }

        // Reduce to bidiagonal form
        int its, jj, k, l, nm;
        double rv1[] = new double[U.colCount];
        double anorm, c, f, g, h, s, scale, x, y, z;
        anorm = 0.0;
        g = 0.0;
        scale = 0.0;
        for (i = 0; i < U.colCount; i++)
        {
            l = i + 2;
            rv1[i] = scale * g;
            g = 0.0;
            scale = 0.0;
            s = 0.0;
            if (i < U.rowCount)
            {
                for (k = i; k < U.rowCount; k++)
                    scale += Math.abs(U.elements[k][i]);

                if (scale != 0.0)
                {
                    for (k = i; k < U.rowCount; k++)
                    {
                        U.elements[k][i] /= scale;
                        s += U.elements[k][i] * U.elements[k][i];
                    }

                    f = U.elements[i][i];
                    if (f >= 0.0)
                        g = -Math.sqrt(s);
                    else
                        g = Math.sqrt(s);

                    h = f * g - s;
                    U.elements[i][i] = f - g;

                    for (j = l - 1; j < U.colCount; j++)
                    {
                        s = 0.0;
                        for (k = i; k < U.rowCount; k++)
                            s += U.elements[k][i] * U.elements[k][j];
                        f = s / h;
                        for (k = i; k < U.rowCount; k++)
                            U.elements[k][j] += f * U.elements[k][i];
                    }
                    for (k = i; k < U.rowCount; k++)
                        U.elements[k][i] *= scale;
                }
            }

            W.elements[i][i] = scale * g;
            g = 0.0;
            s = 0.0;
            scale = 0.0;

            if (i < U.rowCount && i != U.colCount - 1)
            {
                for (k = l - 1; k < U.colCount; k++)
                    scale += Math.abs(U.elements[i][k]);

                if (scale != 0.0)
                {
                    for (k = l - 1; k < U.colCount; k++)
                    {
                        U.elements[i][k] /= scale;
                        s += U.elements[i][k] * U.elements[i][k];
                    }

                    f = U.elements[i][l - 1];
                    if (f >= 0.0)
                        g = -Math.sqrt(s);
                    else
                        g = Math.sqrt(s);

                    h = f * g - s;
                    U.elements[i][l - 1] = f - g;
                    
                    for (k = l - 1; k < U.colCount; k++)
                        rv1[k] = U.elements[i][k] / h;

                    for (j = l - 1; j < U.rowCount; j++)
                    {
                        s = 0.0;
                        for (k = l - 1; k < U.colCount; k++)
                            s += U.elements[j][k] * U.elements[i][k];
                        for (k = l - 1; k < U.colCount; k++)
                            U.elements[j][k] += s * rv1[k];
                    }

                    for (k = l - 1; k < U.colCount; k++)
                        U.elements[i][k] *= scale;
                }
            }

            anorm = Math.max(anorm,
                    Math.abs(W.elements[i][i]) + Math.abs(rv1[i]));
        }

        // Accumulation of right-hand transforms
        l = 0;// This value isn't used, but it avoids a java compiler warning
        for (i = U.colCount - 1; i >= 0; i--)
        {
            if (i < U.colCount - 1)
            {
                if (g != 0.0)
                {
                    for (j = l; j < U.colCount; j++)
                        V.elements[j][i] =
                                (U.elements[i][j] / U.elements[i][l])
                                / g;

                    for (j = l; j < U.colCount; j++)
                    {
                        s = 0.0;
                        for (k = l; k < U.colCount; k++)
                            s += U.elements[i][k] * V.elements[k][j];

                        for (k = l; k < U.colCount; k++)
                            V.elements[k][j] += s * V.elements[k][i];
                    }
                }

                for (j = l; j < U.colCount; j++)
                {
                    V.elements[i][j] = 0.0;
                    V.elements[j][i] = 0.0;
                }
            }
            V.elements[i][i] = 1.0;
            g = rv1[i];
            l = i;
        }

        // Accumulation of left-hand transforms
        for (i = Math.min(U.rowCount, U.colCount) - 1; i >= 0; i--)
        {
            l = i + 1;
            g = W.elements[i][i];
            for (j = l; j < U.colCount; j++)
                U.elements[i][j] = 0.0;

            if (g != 0.0)
            {
                g = 1.0 / g;
                for (j = l; j < U.colCount; j++)
                {
                    s = 0.0;
                    for (k = l; k < U.rowCount; k++)
                        s += U.elements[k][i] * U.elements[k][j];

                    f = (s / U.elements[i][i]) * g;

                    for (k = i; k < U.rowCount; k++)
                        U.elements[k][j] += f * U.elements[k][i];
                }

                for (j = i; j < U.rowCount; j++)
                    U.elements[j][i] *= g;
            }
            else
            {
                for (j = i; j < U.rowCount; j++)
                    U.elements[j][i] = 0.0;
            }
            U.elements[i][i]++;
        }

        // Diagonalization of the bidiagonal form
        boolean finished;
        double eps = 1e-6;
        for (k = U.colCount - 1; k >= 0; k--)
        {
            for (its = 0; its < 30; its++)
            {
                finished = false;
                nm = 0;// This value isn't used, but it avoids compiler warnings
                for (l = k; l >= 0; l--)
                {
                    nm = l - 1;
                    if (l == 0 || Math.abs(rv1[l]) <= eps * anorm)
                    {
                        finished = true;
                        break;
                    }

                    if (Math.abs(W.elements[nm][nm]) <= eps * anorm)
                        break;
                }

                if (!finished)
                {
                    c = 0.0;
                    s = 1.0;
                    for (i = l; i <= k; i++)
                    {
                        f = s * rv1[i];
                        rv1[i] = c * rv1[i];

                        if (Math.abs(f) <= eps * anorm)
                            break;

                        g = W.elements[i][i];
                        h = pythag(f, g);
                        W.elements[i][i] = h;
                        h = 1.0 / h;
                        c = g * h;
                        s = -f * h;
                        for (j = 0; j < U.rowCount; j++)
                        {
                            y = U.elements[j][nm];
                            z = U.elements[j][i];
                            U.elements[j][nm] = y * c + z * s;
                            U.elements[j][i] = z * c - y * s;
                        }
                    }
                }

                z = W.elements[k][k];
                if (l == k)
                {
                    if (z < 0.0)
                    {
                        W.elements[k][k] = -z;
                        for (j = 0; j < U.colCount; j++)
                            V.elements[j][k] = -V.elements[j][k];
                    }
                    break;
                }

                // Print an error if we've hit the iteration limit
				if (its == 29)
					System.err.println(
							"Warning!  Iteration limit reached while" +
							" inverting matrix");

                x = W.elements[l][l];
                nm = k - 1;
                y = W.elements[nm][nm];
                g = rv1[nm];
                h = rv1[k];
                f = ((y-z) * (y+z) + (g-h) * (g+h)) / (2.0 * h * y);
                g = pythag(f, 1.0);
                if (f >= 0.0)
                    f = ((x-z) * (x+z) + h * ((y / (f + Math.abs(g))) - h)) / x;
                else
                    f = ((x-z) * (x+z) + h * ((y / (f - Math.abs(g))) - h)) / x;

                c = 1.0;
                s = 1.0;
                for (j = l; j <= nm; j++)
                {
                    i = j + 1;
                    g = rv1[i];
                    y = W.elements[i][i];
                    h = s * g;
                    g = c * g;
                    z = pythag(f,h);
                    rv1[j] = z;
                    c = f / z;
                    s = h / z;
                    f = x * c + g * s;
                    g = g * c - x * s;
                    h = y * s;
                    y *= c;

                    for (jj = 0; jj < U.colCount; jj++)
                    {
                        x = V.elements[jj][j];
                        z = V.elements[jj][i];
                        V.elements[jj][j] = x * c + z * s;
                        V.elements[jj][i] = z * c - x * s;
                    }

                    z = pythag(f, h);
                    W.elements[j][j] = z;

                    if (z != 0.0)
                    {
                        z = 1.0 / z;
                        c = f * z;
                        s = h * z;
                    }

                    f = c * g + s * y;
                    x = c * y - s * g;

                    for (jj = 0; jj < U.rowCount; jj++)
                    {
                        y = U.elements[jj][j];
                        z = U.elements[jj][i];
                        U.elements[jj][j] = y * c + z * s;
                        U.elements[jj][i] = z * c - y * s;
                    }
                }

                rv1[l] = 0.0;
                rv1[k] = f;
                W.elements[k][k] = x;
            }
        }

        // Remove zero-value singular values and the corresponding columns and
        // rows from the U and V matrices
        // Without this, the results are close, but this makes them much better
        for (i = 0; i < W.colCount; i++)
        {
            // No need to use Math.abs - we've already ensured positive values
            if (W.elements[i][i] < nearlyZero)
            {
                W.elements[i][i] = 1.0;
                U.elements[i][i] = 0.0;
                V.elements[i][i] = 0.0;
            }
        }

        // Some testing aids
        Matrix IfromU = U.GetTranspose().Multiply(U);
        Matrix IfromV = V.GetTranspose().Multiply(V);
        Matrix IfromV2 = V.Multiply(V.GetTranspose());
        Matrix originalAgain = U.Multiply(W).Multiply(V.GetTranspose());

        // Invert the components of W along the diagonal
        for (i = 0; i < W.colCount; i++)
            W.elements[i][i] = 1.0 / W.elements[i][i];

        return V.Multiply(W).Multiply(U.GetTranspose());
    }

	/**
	 * Returns the sign-corrected third-leg dimension of a triangle.  Helper
	 * method for SVD calculation in Psuedoinverse().
	 *
	 * @param a
	 * @param b
	 * @return c
	 */
    private static double pythag(double a, double b)
    {
        double absa = Math.abs(a);
        double absb = Math.abs(b);

        if (absa > absb)
            return absa * Math.sqrt(1.0 + absb * absb / (absa * absa));
        else if (absb == 0.0)
            return 0.0;

        return absb * Math.sqrt(1.0 + absa * absa / (absb * absb));
    }

    /**
	 * Returns the transpose of the matrix.
	 *
	 * @return Transpose of the matrix
	 */
    public Matrix GetTranspose()
    {
        return Transpose(this);
    }

	/**
	 * Returns the transpose of the specified matrix.  Static version of
	 * GetTranspose().
	 *
	 * @param target	Matrix to transpose
	 *
	 * @return Transposition of the specified matrix
	 */
    static public Matrix Transpose(Matrix target)
    {
        // Swap rows for columns
        Matrix transpose = new Matrix(target.colCount, target.rowCount);
        
        int i, j;
        for (i = 0; i < transpose.rowCount; i++)
        {
            for (j = 0; j < transpose.colCount; j++)
                transpose.elements[i][j] = target.elements[j][i];
        }

        return transpose;
    }

	/**
	 * Returns the 2D rotation matrix to rotate a vector counter-clockwise by
	 * the specified angle.
	 *
	 * @param angle	Angle through which the rotation is to occur [rad]
	 *
	 * @return The 2D rotation matrix for the specified angle
	 */
    static public Matrix Get2DRotationMatrix(double angle)
    {
        double rotationArray[][] = new double[][]
                {{Math.cos(angle), -Math.sin(angle)},
                 {Math.sin(angle), Math.cos(angle)}};
        Matrix rotationMatrix = new Matrix(rotationArray);

        return rotationMatrix;
    }
    
    /**
     * Creates a rectangular identity matrix.
     * 
     * @param _rows
     * @param _cols
     * @return identity matrix of specified dimensions
     */
    static public Matrix GetIdentity(int rows, int cols)
    {
        Matrix m = new Matrix(rows, cols);
        m.MakeIdentity();
        
        return m;
    }
    
    /**
     * Creates a square identity matrix.
     * 
     * @param dim
     * @return identity matrix of specified dimensions
     */
    static public Matrix GetIdentity(int dim)
    {
        return GetIdentity(dim, dim);
    }
    
    /**
     * Makes this matrix an identity matrix (overwrites existing contents)
     */
    public void MakeIdentity()
    {
        int i, j;
        for (i = 0; i < rowCount; i++)
        {
            for (j = 0; j < colCount; j++)
            {
                if (i == j)
                    elements[i][j] = 1.0;
                else
                    elements[i][j] = 0.0;
            }
        }
    }
    
    /**
     * Returns a rectangular zero matrix.
     * 
     * @param rows
     * @param cols
     * @return zero matrix of the specified dimensions
     */
    static public Matrix GetZero(int rows, int cols)
    {
        Matrix m = new Matrix(rows, cols);
        m.MakeZero();
        
        return m;
    }
    
    /**
     * Returns a square zero matrix
     * 
     * @param dim
     * @return zero matrix of the specified dimensions
     */
    static public Matrix GetZero(int dim)
    {
        return GetZero(dim, dim);
    }
    
    /**
     * Makes this matrix a zero matrix (overwrites existing contents)
     */
    public void MakeZero()
    {
        int i, j;
        for (i = 0; i < rowCount; i++)
        {
            for (j = 0; j < colCount; j++)
            {
                    elements[i][j] = 0.0;
            }
        }
    }
	
	/**
	 * Method for printing the matrix contents to a string.
	 * 
	 * @return string representing matrix contents
	 */
	public String Print()
	{
		String s = "";
		int i, j;
		for (i = 0; i < rowCount; i++)
		{
			s += "[";
			for (j = 0; j < colCount; j++)
			{
				s += elements[i][j];
				if (j < colCount - 1)
					s += ";\t";
			}
			
			s += "]";
			if (i < rowCount - 1)
				s += "\n";
		}
		
		return s;
	}
}