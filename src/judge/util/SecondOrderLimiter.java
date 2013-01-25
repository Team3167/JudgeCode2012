/*******************************************************************************
* File:  SecondOrderLimiter.java
* Date:  1/10/2011
* Auth:  K. Loux
* Desc:  Velocity and acceleration limiter class.
*******************************************************************************/

// Declare our package
package judge.util;

/**
 * Object that limits the magnitude of a signal and it's first derivative.  Can
 * provide different limits for minimum and maximum.
 *
 * @author K. Loux
 */
public class SecondOrderLimiter
{
    // Fields
    private double vMax, vMin, aMax, aMin;// [units/sec]
    private double lastVel = 0.0;// [units/sec]
    private double freq;// [Hz]

    private SecondOrderFilter filter;

    // Methods
	/**
	 * Constructor for limiting signals symmetrically around zero.
	 *
	 * @param _vMax	Limit on the magnitude of the signal itself
	 * @param _aMax Limit on the magnitude of the signal's first derivative
	 * @param _freq Fixed frequency at which the limiter will be applied [Hz]
	 */
    public SecondOrderLimiter(double _vMax, double _aMax, double _freq)
    {
        // Assign the limits
        vMin = -_vMax;
        vMax = _vMax;
        aMin = -_aMax;
        aMax = _aMax;

        // Assign the frequency
        freq = _freq;
    }

	/**
	 * Constructor for limiting signals with different values for minimums and
	 * maximums.
	 *
	 * @param _vMin Lower limit on the signal itself
	 * @param _vMax Upper limit on the signal itself
	 * @param _aMin Lower limit on the first derivative of the signal
	 * @param _aMax Upper limit on the first derivative of the signal
	 * @param _freq Fixed frequency at which the limiter will be applied [Hz]
	 */
    public SecondOrderLimiter(double _vMin, double _vMax, double _aMin,
            double _aMax, double _freq)
    {
        // Assign the limits
        vMin = _vMin;
        vMax = _vMax;
        aMin = _aMin;
        aMax = _aMax;

        // Assign the frequency
        freq = _freq;
    }

	/**
	 * Returns a signal complying with the limits specified for this object when
	 * it was constructed.  MUST be called at a fixed frequency equal to that
	 * specified in the constructor.
	 *
	 * @param newV	Unlimited signal
	 *
	 * @return Signal following the input signal, but complying with limits
	 * (units match input units)
	 */
    public double Process(double newV)
    {
        // Compute the acceleration
        double newA = (newV - lastVel) * freq;

        // Apply the limit to our acceleration
        if (newA > aMax)
            newA = aMax;
        else if (newA < aMin)
            newA = aMin;

        // Apply the limit to our velocity
        if (lastVel + newA / freq > vMax)
            newV = vMax;
        else if (lastVel + newA / freq < vMin)
            newV = vMin;
        else
            newV = lastVel + newA / freq;

        // Update fields
        lastVel = newV;

        // Apply a filter to the output (if it exists)
        if (filter != null)
            newV = filter.Apply(newV);

        return newV;
    }

    // Adds optional filter on top of the smoothing we're already doing
	/**
	 * Enables optional low-pass filtering (smoothing) on top of basic limiting.
	 *
	 * @param omega	Filter cutoff frequency [Hz]
	 * @param zeta	Filter damping ratio [-]
	 *
	 * @see SecondOrderFilter
	 */
    public void EnableFiltering(double omega, double zeta)
    {
        // Create the filter
        filter = new SecondOrderFilter(omega, zeta, freq);
    }
}