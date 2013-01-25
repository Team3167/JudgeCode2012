/*******************************************************************************
* File:  Wheel.java
* Date:  1/12/2011
* Auth:  K. Loux
* Desc:  Class to handle functions related to a wheel.  Robots should have one
*        of these for each motor powering a wheel (so if there is one motor for
*        two wheels, just use one of these objects).
*******************************************************************************/

// Declare our package
package judge.drive;

// WPI imports
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Encoder;

// Judge imports
import judge.util.PIDControllerII;
import judge.util.SecondOrderFilter;

/**
 * Class representing the drivetrain from one motor to a wheel (or wheels).
 * Handles gearboxes, encoders being mid-stream in the drivetrain (a la
 * toughboxes), and allows for open- or closed-loop control.  Feedback can be
 * filtered prior to use, and necessary motor watchdogs are implemented (even
 * though they are also implemented in the Jaguar object used by this class to
 * turn the wheel).
 * 
 * @author K. Loux
 */
public class Wheel implements MotorSafety
{
    // Fields
	protected MotorSafetyHelper safetyHelper;

    // Type of controller to use for each wheel
    private static final byte CONTROL_OPEN_LOOP = 0;
    private static final byte CONTROL_CLOSED_LOOP = 1;
    
    // Position of wheel center point in robot coordinates
    private final double posX, posY;// [length]

    // Direction of axis pabout which the wheel rotates
    private final double axisX, axisY;

    // Angle of rollers on wheel relative to the wheel's rotation axis
	// NOTE:  This applies to lower rollers (contacting the ground), not the
	// easy-to-see rollers on the top of the wheel.
    private final double rollerAngle;// [deg]

    // The radius of the wheel
    private final double radius;// [length]

	// The rate at which the loop is closed
	private final double freq;// [Hz]

    // The controller object and control type
    private PIDControllerII controller;
    private final byte controlType;

    // The motor object
    private final Jaguar motor;

	// Maximum allowable speed for this wheel
	private final double maxRotationRate;// [rad/sec at the wheel]

    // The encoder object
    private Encoder encoder;
    private double wheelVelocity;// [rad/sec at the wheel]
	private double lastPosition;// [rad]
	private SecondOrderFilter rateFilter;

	private final double stoppedSpeedThreshold = 0.01;// [rad/sec]

    // Methods
	/**
	 * Open loop constructor.
	 *
	 * @param _posX			X-position of the wheel [in]
	 * @param _posY			Y-position of the wheel [in]
	 * @param _axisX		X-component of axis of rotation unit vector (NOTE:
	 * The direction of the axis of rotation defines the positive direction of
	 * rotation for the wheel according to the Right Hand Rule) [-]
	 * @param _axisY		Y-component of axis of rotation unit vector (NOTE:
	 * The direction of the axis of rotation defines the positive direction of
	 * rotation for the wheel according to the Right Hand Rule) [-]
	 * @param _rollerAngle	Angle between the roller in contact with the ground
	 * and the axis of rotation, sign given by Right Hand Rule [deg]
	 * @param _radius		Radius of the wheel [in]
	 * @param motorSlot		cRIO slot into which the digital sidecar is
	 * connected
	 * @param motorChannel	PWM channel on the digital sidecar into which the
	 * Jaguar is connected
     * @param maxSpeed      Maximum speed achievable for this wheel [rad/sec]
	 */
    public Wheel(double _posX, double _posY, double _axisX, double _axisY,
            double _rollerAngle, double _radius,
            int motorSlot, int motorChannel, double maxSpeed)
    {
        // Assign geometry variables
        posX = _posX;
        posY = _posY;
        rollerAngle = _rollerAngle;
        radius = _radius;

        maxRotationRate = maxSpeed;

        controlType = CONTROL_OPEN_LOOP;

        // Normalize the axis direction before making the assignment
        double axisMag = Math.sqrt(_axisX * _axisX + _axisY * _axisY);
        axisX = _axisX / axisMag;
        axisY = _axisY / axisMag;

        // Create the motor object
        motor = new Jaguar(motorSlot, motorChannel);
        motor.set(0.0);

		// Assign a nonsense values to unneeded final fields
		freq = 0.0;

		// Set up the motor safety object (watchdog stuff)
		SetupMotorSafety();
    }

	/**
	 * Closed-loop PI control constructor, anti-windup handled by fixed integral
	 * time.
	 *
	 * @param _posX				X-position of the wheel [in]
	 * @param _posY				Y-position of the wheel [in]
	 * @param _axisX			X-component of axis of rotation unit vector
	 * (NOTE: The direction of the axis of rotation defines the positive
	 * direction of rotation for the wheel according to the Right Hand Rule) [-]
	 * @param _axisY			Y-component of axis of rotation unit vector
	 * (NOTE: The direction of the axis of rotation defines the positive
	 * direction of rotation for the wheel according to the Right Hand Rule) [-]
	 * @param _rollerAngle		Angle between the roller in contact with the
	 * ground and the axis of rotation, sign given by Right Hand Rule [deg]
	 * @param _radius			Radius of the wheel [in]
	 * @param gearRatio			Gear ratio between the wheel and the encoder
	 * (not between the wheel and the motor) [encoder revs/wheel rev]
	 * @param motorSlot			cRIO slot into which the digital sidecar is
	 * connected
	 * @param motorChannel		PWM channel on the digital sidecar into which
	 * the Jaguar is connected
	 * @param _maxRotationRate	Maximum rotation rate of the wheel (not the
	 * motor, nor the encoder) [rad/sec]
	 * @param Kp				Proportional gain
	 * @param Ki				Integral gain
	 * @param queueSize			Number of error samples used to compute the
	 * integral of the error [-]
	 * @param omega				Cutoff frequency for feedback filter [Hz]
	 * @param zeta				Damping ratio for feedback filter [-]
	 * @param _freq				Fixed frequency at which the loop is closed [Hz]
	 * @param encSlotA			cRIO slot into which the digital sidecar
	 * measuring the encoder's A pulse signal is connected
	 * @param encChanA			Digital input channel into which the encoder's A
	 * pulse signal is connected
	 * @param encSlotB			cRIO slot into which the digtial sidecar
	 * measuring the encoder's B pulse signal is connected
	 * @param encChanB			Digital input channel into which the encoder's B
	 * pulse signal is connected
	 * @param encPPR			Encoder pulses per revolution per channel
	 * @param reverseEncoder	Flag indicating whether or not the positive
	 * direction of the encoer should be swapped
	 */
    public Wheel(double _posX, double _posY, double _axisX, double _axisY,
            double _rollerAngle, double _radius, double gearRatio,
			int motorSlot, int motorChannel, double _maxRotationRate,
			double Kp, double Ki, int queueSize,
			double omega, double zeta, double _freq,
            int encSlotA, int encChanA, int encSlotB, int encChanB,
            int encPPR, boolean reverseEncoder)
    {
        // Assign geometry variables
        posX = _posX;
        posY = _posY;
        rollerAngle = _rollerAngle;
        radius = _radius;
		freq = _freq;
		maxRotationRate = _maxRotationRate;

        controlType = CONTROL_CLOSED_LOOP;

		// Create the filter object
		rateFilter = new SecondOrderFilter(omega, zeta, freq);

        // Normalize the axis direction before making the assignment
        double axisMag = Math.sqrt(_axisX * _axisX + _axisY * _axisY);
        axisX = _axisX / axisMag;
        axisY = _axisY / axisMag;

        // Create the motor object
        motor = new Jaguar(motorSlot, motorChannel);
        motor.set(0.0);

        // Create the encoder object
        encoder = new Encoder(encSlotA, encChanA, encSlotB, encChanB,
                reverseEncoder);
        encoder.setDistancePerPulse(2.0 * Math.PI / encPPR / gearRatio);// Disance in rad at wheel
        //encoder.setMaxPeriod(60.0 / (5.0 * encPPR));// Based on 5 motor RPM
        encoder.reset();
        encoder.start();

        // Create controller object
        controller = new PIDControllerII(Kp, Ki, queueSize, freq);

		// Set up the motor safety object (watchdog stuff)
		SetupMotorSafety();
    }

	/**
	 * Closed-loop PI control constructor, anti-windup handled by integral
	 * saturation.
	 *
	 * @param _posX				X-position of the wheel [in]
	 * @param _posY				Y-position of the wheel [in]
	 * @param _axisX			X-component of axis of rotation unit vector
	 * (NOTE: The direction of the axis of rotation defines the positive
	 * direction of rotation for the wheel according to the Right Hand Rule) [-]
	 * @param _axisY			Y-component of axis of rotation unit vector
	 * (NOTE: The direction of the axis of rotation defines the positive
	 * direction of rotation for the wheel according to the Right Hand Rule) [-]
	 * @param _rollerAngle		Angle between the roller in contact with the
	 * ground and the axis of rotation, sign given by Right Hand Rule [deg]
	 * @param _radius			Radius of the wheel [in]
	 * @param gearRatio			Gear ratio between the wheel and the encoder
	 * (not between the wheel and the motor) [encoder revs/wheel rev]
	 * @param motorSlot			cRIO slot into which the digital sidecar is
	 * connected
	 * @param motorChannel		PWM channel on the digital sidecar into which
	 * the Jaguar is connected
	 * @param _maxRotationRate	Maximum rotation rate of the wheel (not the
	 * motor, nor the encoder) [rad/sec]
	 * @param Kp				Proportional gain
	 * @param Ki				Integral gain
	 * @param saturation		Maximum allowable magnitude of the integral of
	 * the error signal
	 * @param omega				Cutoff frequency for feedback filter [Hz]
	 * @param zeta				Damping ratio for feedback filter [-]
	 * @param _freq				Fixed frequency at which the loop is closed [Hz]
	 * @param encSlotA			cRIO slot into which the digital sidecar
	 * measuring the encoder's A pulse signal is connected
	 * @param encChanA			Digital input channel into which the encoder's A
	 * pulse signal is connected
	 * @param encSlotB			cRIO slot into which the digtial sidecar
	 * measuring the encoder's B pulse signal is connected
	 * @param encChanB			Digital input channel into which the encoder's B
	 * pulse signal is connected
	 * @param encPPR			Encoder pulses per revolution per channel
	 * @param reverseEncoder	Flag indicating whether or not the positive
	 * direction of the encoer should be swapped
	 */
    public Wheel(double _posX, double _posY, double _axisX, double _axisY,
            double _rollerAngle, double _radius, double gearRatio,
			int motorSlot, int motorChannel, double _maxRotationRate,
			double Kp, double Ki, double saturation,
			double omega, double zeta, double _freq,
            int encSlotA, int encChanA, int encSlotB, int encChanB,
            int encPPR, boolean reverseEncoder)
    {
        // Assign geometry variables
        posX = _posX;
        posY = _posY;
        rollerAngle = _rollerAngle;
        radius = _radius;
		freq = _freq;
		maxRotationRate = _maxRotationRate;

        controlType = CONTROL_CLOSED_LOOP;

		// Create the filter object
		rateFilter = new SecondOrderFilter(omega, zeta, freq);

        // Normalize the axis direction before making the assignment
        double axisMag = Math.sqrt(_axisX * _axisX + _axisY * _axisY);
        axisX = _axisX / axisMag;
        axisY = _axisY / axisMag;

        // Create the motor object
        motor = new Jaguar(motorSlot, motorChannel);
        motor.set(0.0);

        // Create the encoder object
        encoder = new Encoder(encSlotA, encChanA, encSlotB, encChanB,
                reverseEncoder);
        encoder.setDistancePerPulse(2.0 * Math.PI / encPPR / gearRatio);// Disance in rad at wheel
        //encoder.setMaxPeriod(60.0 / (5.0 * encPPR));// Based on 5 motor RPM
        encoder.reset();
        encoder.start();

        // Create controller object
        controller = new PIDControllerII(Kp, Ki, saturation, freq);

		// Set up the motor safety object (watchdog stuff)
		SetupMotorSafety();
    }

    /**
	 * Closed-loop PID control constructor, anti-windup handled by fixed
	 * integral time.
	 *
	 * @param _posX				X-position of the wheel [in]
	 * @param _posY				Y-position of the wheel [in]
	 * @param _axisX			X-component of axis of rotation unit vector
	 * (NOTE: The direction of the axis of rotation defines the positive
	 * direction of rotation for the wheel according to the Right Hand Rule) [-]
	 * @param _axisY			Y-component of axis of rotation unit vector
	 * (NOTE: The direction of the axis of rotation defines the positive
	 * direction of rotation for the wheel according to the Right Hand Rule) [-]
	 * @param _rollerAngle		Angle between the roller in contact with the
	 * ground and the axis of rotation, sign given by Right Hand Rule [deg]
	 * @param _radius			Radius of the wheel [in]
	 * @param gearRatio			Gear ratio between the wheel and the encoder
	 * (not between the wheel and the motor) [encoder revs/wheel rev]
	 * @param motorSlot			cRIO slot into which the digital sidecar is
	 * connected
	 * @param motorChannel		PWM channel on the digital sidecar into which
	 * the Jaguar is connected
	 * @param _maxRotationRate	Maximum rotation rate of the wheel (not the
	 * motor, nor the encoder) [rad/sec]
	 * @param Kp				Proportional gain
	 * @param Ki				Integral gain
	 * @param Kd				Derivative gain
	 * @param queueSize			Number of error samples used to compute the
	 * integral of the error [-]
	 * @param omega				Cutoff frequency for feedback filter [Hz]
	 * @param zeta				Damping ratio for feedback filter [-]
	 * @param _freq				Fixed frequency at which the loop is closed [Hz]
	 * @param encSlotA			cRIO slot into which the digital sidecar
	 * measuring the encoder's A pulse signal is connected
	 * @param encChanA			Digital input channel into which the encoder's A
	 * pulse signal is connected
	 * @param encSlotB			cRIO slot into which the digtial sidecar
	 * measuring the encoder's B pulse signal is connected
	 * @param encChanB			Digital input channel into which the encoder's B
	 * pulse signal is connected
	 * @param encPPR			Encoder pulses per revolution per channel
	 * @param reverseEncoder	Flag indicating whether or not the positive
	 * direction of the encoer should be swapped
	 */
    public Wheel(double _posX, double _posY, double _axisX, double _axisY,
            double _rollerAngle, double _radius, double gearRatio,
			int motorSlot, int motorChannel, double _maxRotationRate,
            double Kp, double Ki, double Kd,
            int queueSize, double omega, double zeta, double _freq,
            int encSlotA, int encChanA, int encSlotB, int encChanB,
            int encPPR, boolean reverseEncoder)
    {
        // Assign geometry variables
        posX = _posX;
        posY = _posY;
        rollerAngle = _rollerAngle;
        radius = _radius;
		freq = _freq;
		maxRotationRate = _maxRotationRate;

        controlType = CONTROL_CLOSED_LOOP;

		// Create the filter object
		rateFilter = new SecondOrderFilter(omega, zeta, freq);

        // Normalize the axis direction before making the assignment
        double axisMag = Math.sqrt(_axisX * _axisX + _axisY * _axisY);
        axisX = _axisX / axisMag;
        axisY = _axisY / axisMag;

        // Create the motor object
        motor = new Jaguar(motorSlot, motorChannel);
        motor.set(0.0);

        // Create the encoder object
        encoder = new Encoder(encSlotA, encChanA, encSlotB, encChanB,
                reverseEncoder);
        encoder.setDistancePerPulse(2.0 * Math.PI / encPPR / gearRatio);// Disance in rad at wheel
        //encoder.setMaxPeriod(60.0 / (5.0 * encPPR));// Based on 5 motor RPM
        encoder.reset();
        encoder.start();

        // Create controller object
        controller = new PIDControllerII(Kp, Ki, Kd,
                queueSize, omega, zeta, freq);

		// Set up the motor safety object (watchdog stuff)
		SetupMotorSafety();
    }

	/**
	 * Closed-loop PID control constructor, anti-windup handled by integral
	 * saturation.
	 *
	 * @param _posX				X-position of the wheel [in]
	 * @param _posY				Y-position of the wheel [in]
	 * @param _axisX			X-component of axis of rotation unit vector
	 * (NOTE: The direction of the axis of rotation defines the positive
	 * direction of rotation for the wheel according to the Right Hand Rule) [-]
	 * @param _axisY			Y-component of axis of rotation unit vector
	 * (NOTE: The direction of the axis of rotation defines the positive
	 * direction of rotation for the wheel according to the Right Hand Rule) [-]
	 * @param _rollerAngle		Angle between the roller in contact with the
	 * ground and the axis of rotation, sign given by Right Hand Rule [deg]
	 * @param _radius			Radius of the wheel [in]
	 * @param gearRatio			Gear ratio between the wheel and the encoder
	 * (not between the wheel and the motor) [encoder revs/wheel rev]
	 * @param motorSlot			cRIO slot into which the digital sidecar is
	 * connected
	 * @param motorChannel		PWM channel on the digital sidecar into which
	 * the Jaguar is connected
	 * @param _maxRotationRate	Maximum rotation rate of the wheel (not the
	 * motor, nor the encoder) [rad/sec]
	 * @param Kp				Proportional gain
	 * @param Ki				Integral gain
	 * @param Kd				Derivative gain
	 * @param saturation		Maximum allowable magnitude of the integral of
	 * the error signal
	 * @param omega				Cutoff frequency for feedback filter [Hz]
	 * @param zeta				Damping ratio for feedback filter [-]
	 * @param _freq				Fixed frequency at which the loop is closed [Hz]
	 * @param encSlotA			cRIO slot into which the digital sidecar
	 * measuring the encoder's A pulse signal is connected
	 * @param encChanA			Digital input channel into which the encoder's A
	 * pulse signal is connected
	 * @param encSlotB			cRIO slot into which the digtial sidecar
	 * measuring the encoder's B pulse signal is connected
	 * @param encChanB			Digital input channel into which the encoder's B
	 * pulse signal is connected
	 * @param encPPR			Encoder pulses per revolution per channel
	 * @param reverseEncoder	Flag indicating whether or not the positive
	 * direction of the encoer should be swapped
	 */
    public Wheel(double _posX, double _posY, double _axisX, double _axisY,
            double _rollerAngle, double _radius, double gearRatio,
			int motorSlot, int motorChannel, double _maxRotationRate,
            double Kp, double Ki, double Kd,
            double saturation, double omega, double zeta, double _freq,
            int encSlotA, int encChanA, int encSlotB, int encChanB,
            int encPPR, boolean reverseEncoder)
    {
        // Assign geometry variables
        posX = _posX;
        posY = _posY;
        rollerAngle = _rollerAngle;
        radius = _radius;
		freq = _freq;
		maxRotationRate = _maxRotationRate;

        controlType = CONTROL_CLOSED_LOOP;

		// Create the filter object
		rateFilter = new SecondOrderFilter(omega, zeta, freq);

        // Normalize the axis direction before making the assignment
        double axisMag = Math.sqrt(_axisX * _axisX + _axisY * _axisY);
        axisX = _axisX / axisMag;
        axisY = _axisY / axisMag;

        // Create the motor object
        motor = new Jaguar(motorSlot, motorChannel);
        motor.set(0.0);

        // Create the encoder object
        encoder = new Encoder(encSlotA, encChanA, encSlotB, encChanB,
                reverseEncoder);
        encoder.setDistancePerPulse(2.0 * Math.PI / encPPR / gearRatio);// Disance in rad at wheel
        //encoder.setMaxPeriod(60.0 / (5.0 * encPPR));// Based on 5 motor RPM
        encoder.reset();
        encoder.start();

        // Create controller object
        controller = new PIDControllerII(Kp, Ki, Kd,
                saturation, omega, zeta, freq);

		// Set up the motor safety object (watchdog stuff)
		SetupMotorSafety();
    }

	/**
	 * Main control method handling speed measurement and loop closure.  MUST be
	 * called at the fixed rate specified when constructed.
	 *
	 * @param cmdOmega	Commanded wheel speed (not motor, nor encoder speed)
	 * [rad/sec]
	 */
    protected void DoControl(final double cmdOmega)
    {
        double motorPWMCmd;

        // Read from encoder, pass along cmd and feedback to the controller
        // The output is scaled by the maximum velocity so we get an input to
        // the PWM between -1 and 1.
        if (controlType == CONTROL_CLOSED_LOOP)
        {
			double position = encoder.getDistance();// [rad at wheel]
            wheelVelocity = rateFilter.Apply((position - lastPosition) * freq);
            lastPosition = position;
            motorPWMCmd = controller.DoControl(cmdOmega, wheelVelocity);
            //motorPWMCmd = cmdOmega / maxRotationRate;// Uncomment for open loop
        }
        else
        {
            wheelVelocity = cmdOmega;
            motorPWMCmd = cmdOmega / maxRotationRate;
        }

        // Limit the command from -1.0 to 1.0
        if (motorPWMCmd < -1.0)
            motorPWMCmd = -1.0;
        else if (motorPWMCmd > 1.0)
            motorPWMCmd = 1.0;

        // Issue the motor speed command
        motor.set(motorPWMCmd);

		// Feed the watchdog
		safetyHelper.feed();
    }

	/**
	 * Returns the X position of the wheel (specified when constructed).
	 *
	 * @return X position of the wheel [in]
	 */
    public double GetXPos()
    {
        return posX;
    }

	/**
	 * Returns the Y position of the wheel (specified when constructed).
	 *
	 * @return Y position of the wheel [in]
	 */
    public double GetYPos()
    {
        return posY;
    }

	/**
	 * Returns the X component of the unit vector defining the wheel's axis of
	 * rotation (specified when constructed).
	 *
	 * @return X component of the unit vector defining the wheel's axis of
	 * rotation [-]
	 */
    public double GetRotationAxisX()
    {
        return axisX;
    }

	/**
	 * Returns the Y component of the unit vector defining the wheel's axis of
	 * rotation (specified when constructed).
	 *
	 * @return Y component of the unit vector defining the wheel's axis of
	 * rotation [-]
	 */
    public double GetRotationAxisY()
    {
        return axisY;
    }

	/**
	 * Returns the angle between the roller contacting the ground and the
	 * wheel's axis of rotation (specified when constructed).
	 *
	 * @return Angle between the rolling element contacting the ground and the
	 * wheel's axis of rotation [deg]
	 */
    public double GetRollerAngle()
    {
        return rollerAngle;
    }

	/**
	 * Returns the wheel's radius (specified when constructed).
	 *
	 * @return Radius of the wheel [in]
	 */
    public double GetRadius()
    {
        return radius;
    }

	/**
	 * Returns the measured velocity of the wheel (not the motor, nor the
	 * encoder).
	 *
	 * @return Wheel velocity [rad/sec]
	 */
    public double GetWheelVelocity()
    {
        return wheelVelocity;// [rad/sec at the wheel]
    }

	/**
	 * Returns the maximum allowable rotation rate of the wheel.
	 *
	 * @return Maximum allowable magnitude of the wheel speed [rad/sec]
	 */
	public double GetMaxRotationRate()
	{
		return maxRotationRate;// [rad/sec at the wheel]
	}

	/**
	 * Returns the Jaguar object powering this wheel's motor.
	 *
	 * @return Jaguar object powering the associated motor
	 */
	public Jaguar GetMotor()
	{
		return motor;
	}

	/**
	 * Returns the encoder measuring the position of the wheel (or some point in
	 * the drivetrain).
	 *
	 * @return Encoder measuring the rotation of the wheel
	 */
	public Encoder GetEncoder()
	{
		return encoder;
	}

	/**
	 * Resets the controller's error integral and the encoder position and
	 * velocity.
	 */
    public void ResetController()
    {
        controller.ResetError();
        encoder.reset();
        lastPosition = 0.0;
    }

	/**
	 * Returns true if the wheel speed is below the defined threshold.
	 *
	 * @return True if the wheel speed is slow enough to consider stopped
	 */
	public boolean IsStopped()
	{
		if (Math.abs(wheelVelocity) < stoppedSpeedThreshold)
			return true;

		return false;
	}

	// Required overloads for MotorSafety interface ============================
	public void setExpiration(double timeout)
	{
        safetyHelper.setExpiration(timeout);
    }

    public double getExpiration()
	{
        return safetyHelper.getExpiration();
    }

    public boolean isAlive()
	{
        return safetyHelper.isAlive();
    }

    public boolean isSafetyEnabled()
	{
        return safetyHelper.isSafetyEnabled();
    }

    public void setSafetyEnabled(boolean enabled)
	{
        safetyHelper.setSafetyEnabled(enabled);
    }

    public void stopMotor()
	{
		motor.set(0.0);
    }

	private void SetupMotorSafety()
	{
        safetyHelper = new MotorSafetyHelper(this);
        safetyHelper.setExpiration(MotorSafety.DEFAULT_SAFETY_EXPIRATION);
        safetyHelper.setSafetyEnabled(true);
    }
    
    public String getDescription()
    {
        return "Wheel controller " + motor.getChannel()
                + " on module " + motor.getModuleNumber();
    }
}
