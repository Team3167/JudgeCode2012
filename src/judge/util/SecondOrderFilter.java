/*******************************************************************************
* File:  SecondOrderFilter.java
* Date:  1/12/2011
* Auth:  K. Loux
* Desc:  Implements a discrete low-pass second order filter (defined by cutoff
*        frequency and damping ratio).
*******************************************************************************/

// Declare our package
package judge.util;

/**
 * Standard discrete-time second-order low-pass filter.
 *
 * @author K. Loux
 */
public class SecondOrderFilter
{
    // Fields
    private final double a0, a1, a2, b1, b2;
    private double lastIn, lastLastIn, lastOut, lastLastOut;

    // Methods
    // Constructor (omega and freq are in Hz, zeta is unitless)
	/**
	 * Constructor.
	 *
	 * @param omega	Filter cutoff frequency [Hz]
	 * @param zeta	Filter damping ratio [-]
	 * @param freq	Fixed frequency at which filter is applied [Hz]
	 */
    public SecondOrderFilter(double omega, double zeta, double freq)
    {
        // Convert omega to rad/sec and calculate the time step
        double ts = 1.0 / freq;// [sec]
        omega *= 2.0 * Math.PI;// [rad/sec]

        // Compute the filter parameters
        double den = 4.0 + 4.0 * omega * zeta * ts + ts * ts * omega * omega;
        a0 = ts * ts * omega * omega / den;
        a1 = 2.0 * ts * ts * omega * omega / den;
        a2 = a0;

        b1 = 8.0 / den - a1;
        b2 = (4.0 * omega * zeta * ts - ts * ts * omega * omega - 4.0) / den;

        // Initialize variables
        lastIn = 0.0;
        lastLastIn = 0.0;
        lastOut = 0.0;
        lastLastOut = 0.0;
    }

	/**
	 * Returns the filtered signal.  MUST be called at the frequency specified
	 * when constructed.
	 *
	 * @param in	Unfiltered signal
	 *
	 * @return Filtered signal (units match input units)
	 */
    public double Apply(double in)
    {
        double out;

        // Compute output
        out = in * a0 + lastIn * a1 + lastLastIn * a2
                + lastOut * b1 + lastLastOut * b2;
        
        // Store required values
        lastLastOut = lastOut;
        lastOut = out;
        lastLastIn = lastIn;
        lastIn = in;

        return out;
    }
}