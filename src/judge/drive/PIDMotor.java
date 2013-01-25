/*******************************************************************************
* File:  PIDMotor.java
* Date:  1/14/2012
* Auth:  K. Loux
* Desc:  Class to handle functions related to a speed-controlled motor.
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
import judge.RobotConfiguration;

/**
 * Class representing a motor, encoder and the controller.
 * 
 * @author K. Loux
 */
public class PIDMotor implements MotorSafety
{
    // Fields
	protected MotorSafetyHelper safetyHelper;
	
    // The controller object and control type
    private PIDControllerII controller;
	private double maxOutputVelocity;// [rad/sec]

    // The motor object and its parameters
    private final Jaguar motor;
	private /*final*/ double maxMotorVelocity;// [rad/sec]
	
	// Motor types (internal motor libarary)
	public class Motor
	{
		public static final byte motorCIM = 0;
		public static final byte motorFisherPrice = 1;
		public static final byte motorWindow = 2;
		public static final byte motorBaneBotsRS550 = 3;
		public static final byte motorAMGear = 4;
	}

    // The encoder object
    private Encoder encoder;
    private double velocity;// [rad/sec]
	private double lastPosition;// [rad]
	private SecondOrderFilter rateFilter;
	
	// Other important parameters
	//private double motorToLoadGearRatio;// [-]
	//private double motorToEncoderGearRatio;// [-]

	private final double stoppedSpeedThreshold = 0.01;// [rad/sec]

	/**
	 * Constructor.  PI, queue-size limited integral.
	 * 
	 * @param motorSlot
	 * @param motorChannel
	 * @param motorType
	 * @param motorToLoadGearRatio
	 * @param motorToEncoderGearRatio
	 * @param Kp
	 * @param Ki
	 * @param queueSize
	 * @param omega
	 * @param zeta
	 * @param _freq
	 * @param encSlotA
	 * @param encChanA
	 * @param encSlotB
	 * @param encChanB
	 * @param encPPR
	 * @param reverseEncoder 
	 */
    public PIDMotor(int motorSlot, int motorChannel, byte motorType,
			double motorToLoadGearRatio, double motorToEncoderGearRatio,
			double Kp, double Ki, int queueSize,
			double omega, double zeta, double _freq,
            int encSlotA, int encChanA, int encSlotB, int encChanB,
            int encPPR, boolean reverseEncoder)
    {
		// Assign the motor type
		SetMotor(motorType);
		maxOutputVelocity = maxMotorVelocity / motorToLoadGearRatio;
		
		// Create the filter object
		rateFilter = new SecondOrderFilter(omega, zeta,
				RobotConfiguration.frequency);

        // Create the motor object
        motor = new Jaguar(motorSlot, motorChannel);
        motor.set(0.0);

        // Create the encoder object
        encoder = new Encoder(encSlotA, encChanA, encSlotB, encChanB,
                reverseEncoder);
        encoder.setDistancePerPulse(2.0 * Math.PI / encPPR
				* motorToEncoderGearRatio / motorToLoadGearRatio);
        encoder.reset();
        encoder.start();

        // Create controller object
        controller = new PIDControllerII(Kp, Ki, queueSize,
				RobotConfiguration.frequency);

		// Set up the motor safety object (watchdog stuff)
		SetupMotorSafety();
    }

	/**
	 * Constructor.  PI, saturation limited integral.
	 * 
	 * @param motorSlot
	 * @param motorChannel
	 * @param motorType
	 * @param motorToLoadGearRatio
	 * @param motorToEncoderGearRatio
	 * @param Kp
	 * @param Ki
	 * @param saturation
	 * @param omega
	 * @param zeta
	 * @param _freq
	 * @param encSlotA
	 * @param encChanA
	 * @param encSlotB
	 * @param encChanB
	 * @param encPPR
	 * @param reverseEncoder 
	 */
    public PIDMotor(int motorSlot, int motorChannel, byte motorType,
			double motorToLoadGearRatio, double motorToEncoderGearRatio,
			double Kp, double Ki, double saturation,
			double omega, double zeta, double _freq,
            int encSlotA, int encChanA, int encSlotB, int encChanB,
            int encPPR, boolean reverseEncoder)
    {
		// Assign the motor type
		SetMotor(motorType);
		maxOutputVelocity = maxMotorVelocity / motorToLoadGearRatio;
		
		// Create the filter object
		rateFilter = new SecondOrderFilter(omega, zeta,
				RobotConfiguration.frequency);

        // Create the motor object
        motor = new Jaguar(motorSlot, motorChannel);
        motor.set(0.0);

        // Create the encoder object
        encoder = new Encoder(encSlotA, encChanA, encSlotB, encChanB,
                reverseEncoder);
        encoder.setDistancePerPulse(2.0 * Math.PI / encPPR
				* motorToEncoderGearRatio / motorToLoadGearRatio);
        encoder.reset();
        encoder.start();

        // Create controller object
        controller = new PIDControllerII(Kp, Ki, saturation,
				RobotConfiguration.frequency);

		// Set up the motor safety object (watchdog stuff)
		SetupMotorSafety();
    }

	/**
	 * Constructor.  PID, queue-size limited integral.
	 * 
	 * @param motorSlot
	 * @param motorChannel
	 * @param motorType
	 * @param motorToLoadGearRatio
	 * @param motorToEncoderGearRatio
	 * @param Kp
	 * @param Ki
	 * @param Kd
	 * @param queueSize
	 * @param omega
	 * @param zeta
	 * @param _freq
	 * @param encSlotA
	 * @param encChanA
	 * @param encSlotB
	 * @param encChanB
	 * @param encPPR
	 * @param reverseEncoder 
	 */
    public PIDMotor(int motorSlot, int motorChannel, byte motorType,
			double motorToLoadGearRatio, double motorToEncoderGearRatio,
			double Kp, double Ki, double Kd, int queueSize,
			double omega, double zeta, double _freq,
            int encSlotA, int encChanA, int encSlotB, int encChanB,
            int encPPR, boolean reverseEncoder)
    {
        // Assign the motor type
		SetMotor(motorType);
		maxOutputVelocity = maxMotorVelocity / motorToLoadGearRatio;
		
		// Create the filter object
		rateFilter = new SecondOrderFilter(omega, zeta,
				RobotConfiguration.frequency);

        // Create the motor object
        motor = new Jaguar(motorSlot, motorChannel);
        motor.set(0.0);

        // Create the encoder object
        encoder = new Encoder(encSlotA, encChanA, encSlotB, encChanB,
                reverseEncoder);
        encoder.setDistancePerPulse(2.0 * Math.PI / encPPR
				* motorToEncoderGearRatio / motorToLoadGearRatio);
        encoder.reset();
        encoder.start();

        // Create controller object
        controller = new PIDControllerII(Kp, Ki, Kd, omega, zeta, queueSize,
				RobotConfiguration.frequency);

		// Set up the motor safety object (watchdog stuff)
		SetupMotorSafety();
    }

	/**
	 * Constructor.  PID, saturation-limited integral.
	 * 
	 * @param motorSlot
	 * @param motorChannel
	 * @param motorType
	 * @param motorToLoadGearRatio
	 * @param motorToEncoderGearRatio
	 * @param Kp
	 * @param Ki
	 * @param Kd
	 * @param saturation
	 * @param omega
	 * @param zeta
	 * @param _freq
	 * @param encSlotA
	 * @param encChanA
	 * @param encSlotB
	 * @param encChanB
	 * @param encPPR
	 * @param reverseEncoder 
	 */
    public PIDMotor(int motorSlot, int motorChannel, byte motorType,
			double motorToLoadGearRatio, double motorToEncoderGearRatio,
			double Kp, double Ki, double Kd, double saturation,
			double omega, double zeta, double _freq,
            int encSlotA, int encChanA, int encSlotB, int encChanB,
            int encPPR, boolean reverseEncoder)
    {
        // Assign the motor type
		SetMotor(motorType);
		maxOutputVelocity = maxMotorVelocity / motorToLoadGearRatio;
		
		// Create the filter object
		rateFilter = new SecondOrderFilter(omega, zeta,
				RobotConfiguration.frequency);

        // Create the motor object
        motor = new Jaguar(motorSlot, motorChannel);
        motor.set(0.0);

        // Create the encoder object
        encoder = new Encoder(encSlotA, encChanA, encSlotB, encChanB,
                reverseEncoder);
        encoder.setDistancePerPulse(2.0 * Math.PI / encPPR
				* motorToEncoderGearRatio / motorToLoadGearRatio);
        encoder.reset();
        encoder.start();

        // Create controller object
        controller = new PIDControllerII(Kp, Ki, Kd, omega, zeta, saturation,
				RobotConfiguration.frequency);

		// Set up the motor safety object (watchdog stuff)
		SetupMotorSafety();
    }
	
	/**
	 * Sets the motor type and all related motor parameters.
	 * 
	 * @param type
	 * @throws IllegalArgumentException if the specified type is unrecognized
	 */
	private void SetMotor(byte type) throws IllegalArgumentException
	{
		switch(type)
		{
			case Motor.motorCIM:
				maxMotorVelocity = RobotConfiguration.cimMotorMaxSpeed
						* 2.0 * Math.PI / 60.0;
				break;
				
			case Motor.motorFisherPrice:
				maxMotorVelocity = RobotConfiguration.fisherPriceMotorMaxSpeed
						* 2.0 * Math.PI / 60.0;
				break;
				
			case Motor.motorWindow:
				maxMotorVelocity = RobotConfiguration.windowMotorMaxSpeed
						* 2.0 / Math.PI / 60.0;
				break;
				
			case Motor.motorBaneBotsRS550:
				maxMotorVelocity = RobotConfiguration.baneBotsRS550MaxSpeed
						* 2.0 / Math.PI / 60.0;
				break;
					
			case Motor.motorAMGear:
				maxMotorVelocity = RobotConfiguration.amGearMotorMaxSpeed
						* 2.0 / Math.PI / 60.0;
				break;
				
			default:
				throw new IllegalArgumentException("Unknown motor type!");
		}
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
		double position = encoder.getDistance();// [rad at output]
		velocity = rateFilter.Apply((position - lastPosition)
				* RobotConfiguration.frequency);
        lastPosition = position;
		motorPWMCmd = controller.DoControl(cmdOmega, velocity);
		//motorPWMCmd = cmdOmega / maxOutputVelocity;// Uncomment for open loop

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
	 * Returns the measured velocity of the output (calculated through gear
	 * ratios).
	 *
	 * @return measured velocity [rad/sec]
	 */
    public double GetVelocity()
    {
        return velocity;// [rad/sec]
    }

	/**
	 * Returns the maximum allowable rotation rate of the motor.
	 *
	 * @return Maximum allowable magnitude of the motor speed [rad/sec]
	 */
	public double GetMaxMotorVelocity()
	{
		return maxMotorVelocity;// [rad/sec]
	}
	
	/**
	 * Returns the maximum allowable rotation rate of the output shaft.
	 *
	 * @return Maximum allowable magnitude of the output speed [rad/sec]
	 */
	public double GetMaxOutputVelocity()
	{
		return maxOutputVelocity;// [rad/sec]
	}

	/**
	 * Returns the Jaguar object powering the motor.
	 *
	 * @return Jaguar object powering the associated motor
	 */
	public Jaguar GetMotor()
	{
		return motor;
	}

	/**
	 * Returns the encoder measuring the position motor (or some point in
	 * the drivetrain).
	 *
	 * @return Encoder measuring the rotation of the motor
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
		if (Math.abs(velocity) < stoppedSpeedThreshold)
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
        return "Controlled motor " + motor.getChannel()
                + " on module " + motor.getModuleNumber();
    }
}
