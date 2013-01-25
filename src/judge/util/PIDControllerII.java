/*******************************************************************************
* File:  PIDControllerII.java
* Date:  1/9/2011
* Auth:  K. Loux
* Desc:  Generic PID controller implementation.  Can be used to create a closed-
*        loop controller for any system with a feedback mechanism.
*
*        NOTE:  Sample time is not required to use this object, although a
*        consistent sample time is required for proper operation.  Units for Ki
*        and Kd are the same as Kp, because the actual integral velocity of the
*        error signal are not computed.
*******************************************************************************/

// Declare our package
package judge.util;

/**
 * PID Controller object with anti-windup mechanisms.  To be used for closing
 * any kind of control loop.
 *
 * @author K. Loux
 */
public class PIDControllerII
{
    // Fields
    // Controller gains
    private double Kp = 1.0, Ki = 0.0, Kd = 0.0;

    // Filter for derivative term
    private SecondOrderFilter derivativeFilter;

    // Error
    private double error = 0.0;

    // Anti-windup mechanism for controller gain
    private final int queueSize;
	private final double saturation;
    private double[] integralQueue;
    private double integralValue = 0.0;
    private int queuePointer = 0;

	// Running frequency
	private final double freq;// [Hz]

    // Methods
	/**
	 * Contructor for PI loops with anti-windup controlled by fixing the
	 * integral time (only remember the most recent error samples).
	 *
	 * @param _Kp			Proportional gain
	 * @param _Ki			Integral gain
	 * @param _queueSize	Number of error samples to hold in the integral
	 * queue
	 * @param _freq			Fixed frequency at which the loop is closed [Hz]
	 */
    public PIDControllerII(double _Kp, double _Ki, int _queueSize, double _freq)
    {
        // Initialize the gains
        Kp = _Kp;
        Ki = _Ki;

		freq = _freq;

        // Initialize the anti-windup objects
        queueSize = _queueSize;
		saturation = 0.0;
        if (queueSize > 0)
            integralQueue = new double[queueSize];
    }

	/**
	 * Constructor for PI loops with anti-windup controlled by saturating the
	 * integral term.
	 *
	 * @param _Kp			Proportional gain
	 * @param _Ki			Integral gain
	 * @param _saturation	Maximum allowable magnitude of the integral of the
	 * error signal
	 * @param _freq			Fixed frequency at which the loop is closed [Hz]
	 */
    public PIDControllerII(double _Kp, double _Ki, double _saturation,
			double _freq)
    {
        // Initialize the gains
        Kp = _Kp;
        Ki = _Ki;

		freq = _freq;

        // Initialize the anti-windup objects
        queueSize = 0;
		saturation = _saturation;
        if (queueSize > 0)
            integralQueue = new double[queueSize];
    }

	/**
	 * Constructor for PID loops with anti-windup handled by fixing the integral
	 * time (only remember the most recent error samples).
	 *
	 * @param _Kp			Proportional gain
	 * @param _Ki			Integral gain
	 * @param _Kd			Derivative gain
	 * @param _queueSize	Number of error samples to hold in the integral
	 * queue
	 * @param omega			Cutoff frequency for optional derivative filter (set
	 * to zero to disable filter)
	 * @param zeta			Damping ratio for optional derivative filter (set to
	 * zero to disable filter)
	 * @param _freq			Fixed frequency at which the loop is closed [Hz]
	 */
    public PIDControllerII(double _Kp, double _Ki, double _Kd,
            int _queueSize, double omega, double zeta, double _freq)
    {
        // Initialize the gains
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;

		freq = _freq;

        // Create the filter object (if specified)
        if (omega > 0 && freq > 0)
            derivativeFilter = new SecondOrderFilter(omega, zeta, freq);

        // Initialize the anti-windup objects
        queueSize = _queueSize;
		saturation = 0.0;
        if (queueSize > 0)
            integralQueue = new double[queueSize];
    }

	/**
	 * Constructor for PID loops with anti-windup handled by saturating the
	 * integral term.
	 *
	 * @param _Kp			Proportional gain
	 * @param _Ki			Integral gain
	 * @param _Kd			Derivative gain
	 * @param _saturation	Maximum allowable magnitude of the integral of the
	 * error signal
	 * @param omega			Cutoff frequency for optional derivative filter (set
	 * to zero to disable filter)
	 * @param zeta			Damping ratio for optional derivative filter (set to
	 * zero to disable filter)
	 * @param _freq			Fixed frequency at which the loop is closed [Hz]
	 */
	public PIDControllerII(double _Kp, double _Ki, double _Kd,
            double _saturation, double omega, double zeta, double _freq)
    {
        // Initialize the gains
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;

		freq = _freq;

        // Create the filter object (if specified)
        if (omega > 0 && freq > 0)
            derivativeFilter = new SecondOrderFilter(omega, zeta, freq);

        // Initialize the anti-windup objects
        queueSize = 0;
		saturation = _saturation;
        if (queueSize > 0)
            integralQueue = new double[queueSize];
    }

	/**
	 * Returns the control signal for the loop.  MUST be called at the frequency
	 * specified when constructed.
	 *
	 * @param command	Reference signal (setpoint)
	 * @param feedback	Feedback signal (measured response)
	 *
	 * @return Control signal
	 */
    public double DoControl(double command, double feedback)
    {
		return DoControl(command - feedback);
    }
	
	public double DoControl(double newError)
    {
		// Calculate the rate of change of error
        double errorDot = (newError - error) * freq;

        // Apply a filter to the derivative term
        if (derivativeFilter != null)
            errorDot = derivativeFilter.Apply(errorDot);

        // Compute the integral and limit the size of the integral to prevent
        // windup (handled in PushIntegral())
        PushIntegral(newError);

        // Assign the new error to our field
        error = newError;

        // Generate the new command signal (input to plant/output of controller)
        // and return it
        return Kp * error + Ki * integralValue + Kd * errorDot;
    }

    // Accessor methods=========================================================
	/**
	 * Returns the current value of the proportional gain.
	 *
	 * @return Proportional gain value
	 */
    public double GetKp()
    {
        return Kp;
    }

	/**
	 * Returns the current value of the integral gain.
	 *
	 * @return Integral gain value
	 */
    public double GetKi()
    {
        return Ki;
    }

	/**
	 * Returns the current value of the derivative gain
	 *
	 * @return Derivative gain value
	 */
    public double GetKd()
    {
        return Kd;
    }

	/**
	 * Returns the current size of the integral queue (if used).
	 *
	 * @return Number of values stored in the integral queue
	 */
    public int GetQueueSize()
    {
        return queueSize;
    }

	/**
	 * Sets the proportional gain to the specified value.
	 *
	 * @param _Kp	New proportional gain
	 */
    public void SetKp(double _Kp)
    {
        Kp = _Kp;
    }

	/**
	 * Sets the integral gain to the specified value.
	 *
	 * @param _Ki	New integral gain
	 */
    public void SetKi(double _Ki)
    {
        Ki = _Ki;
    }

	/**
	 * Sets the derivative gain to the specified value.
	 *
	 * @param _Kd	New derivative gain
	 */
    public void SetKd(double _Kd)
    {
        Kd = _Kd;
    }

	/**
	 * Returns the difference between the reference and feedback signals
	 *
	 * @return Current error measurement
	 */
    public double GetError()
    {
        return error;
    }

	/**
	 * Calculates the integral of the error signal using the specified
	 * anti-windup method (if any).
	 *
	 * @param e	Current error signal
	 */
    private void PushIntegral(double e)
    {
        // Compute the integral
		double increment = e / freq;
        integralValue += increment;

        // Apply anti-windup, if specified
        if (queueSize > 0)
        {
            // Subtract the oldest data from the integral, and store the new
            // data in it's place
            integralValue -= integralQueue[queuePointer];
            integralQueue[queuePointer] = increment;

            // Increment the integral pointer
            queuePointer++;
            if (queuePointer >= queueSize)
                queuePointer = 0;
        }
		else if (saturation > 0)
		{
			if (integralValue > saturation)
				integralValue = saturation;
			else if (integralValue < -saturation)
				integralValue = -saturation;
		}
    }

	/**
	 * Resets the value of the integral of the error signal to zero.
	 */
    public void ResetError()
    {
        integralValue = 0.0;
    }
}