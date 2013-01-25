/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package judge.drive;

// libWPI imports
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Servo;

// Local imports
import judge.RobotConfiguration;

/**
 * Class representing the ball-retrieving bucket.
 *
 * @author kloux
 */
public class Bucket
{
	private Jaguar motor;
	private Jaguar brushMotor;

	private DigitalInput upperNotLimit;
	private DigitalInput lowerNotLimit;

	private Servo bridgeTipper;
	private final double servoOutPosition = 1.00;
	private final double servoInPosition = 0.0;

	private final double brushSpeed = -0.35;
	private boolean tipperExtend = false;

	private final double upSpeed = -1.0;
	private final double downSpeed = 1.0;

	/**
	 * Class storing movement direction flags.
	 */
	public class Move
	{
		public static final byte up = 0;
		public static final byte down = 1;
		public static final byte stop = 2;
	}

	private double speed;// [%]

	/**
	 * Constructor for bucket class.  Creates objects on specified ports.
	 */
	public Bucket()
	{
		motor = new Jaguar(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.bucketMotorChannel);
		brushMotor = new Jaguar(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.brushMotorChannel);

		upperNotLimit = new DigitalInput(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.bucketUpperLimitChannel);
		lowerNotLimit = new DigitalInput(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.bucketLowerLimitChannel);

		bridgeTipper = new Servo(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.bridgeTipperChannel);
	}

	/**
	 * Method to be called at periodic frequency.  Ensures bucket is responding
	 * to driver's commands.
	 */
	public void Update()
	{
		// Only run the brush motor at the very top and very bottom of travel
		// Also, do not run the motor up if we're all the way up,
		// or down if we're all the way down
		if ((upperNotLimit.get() && lowerNotLimit.get()) ||// Limit switches not activated
				(upperNotLimit.get() && speed == upSpeed) ||// Upper limit switch not activated
				(lowerNotLimit.get() && speed == downSpeed))// Lower limit switch not activated
		{
			motor.set(speed);
			brushMotor.set(0.0);
		}
		else
		{
			motor.set(0.0);
			brushMotor.set(brushSpeed);
		}

		if (tipperExtend)
			bridgeTipper.set(servoOutPosition);
		else
			bridgeTipper.set(servoInPosition);
	}

	/**
	 * Sets the direction as determined by the driver.
	 *
	 * @param direction representing either up, down, or stopped
	 */
	public void Set(byte direction)
	{
		switch (direction)
		{
			case Move.up:
				speed = upSpeed;
				//tipperExtend = false;
				break;

			case Move.down:
				speed = downSpeed;
				break;

			case Move.stop:
				speed = 0.0;
				break;
		}
	}

	public void SetExtendBridgeTipper(boolean extend)
	{
		tipperExtend = extend;
	}

	public boolean IsUp()
	{
		return !upperNotLimit.get();
	}

	public boolean IsDown()
	{
		return !lowerNotLimit.get();
	}
}
