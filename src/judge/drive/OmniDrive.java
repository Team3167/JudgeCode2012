/*******************************************************************************
* File:  OmniDrive.java
* Date:  10/16/2012
* Auth:  K. Loux
* Desc:  Drive class for using two regular wheels and two omni-wheels.
*******************************************************************************/

// Declare our package
package judge.drive;

import edu.wpi.first.wpilibj.Joystick;
import judge.util.SecondOrderLimiter;

/**
 *
 * @author kloux
 */
public class OmniDrive
{
	// Fields
    // Frequency at which this object is updated
    private double freq;

    // Velocity and Acceleration Limits
    private SecondOrderLimiter wheelSpeedLimiter[];
    private double friction = 1.0;// [-]

	// Deadband to apply when using joystick
	private double deadband = 0.03;// units are % / 100
	private double minimumOutput = 0.2;// units are % / 100 (open-loop only)
	private double dbSlope;// [-]
	private double dbIntercept;// [-]

	// Array of wheel objects
    private WheelList wheelList;

	public OmniDrive(double _freq, int digitalSideCarModule,
			int analogInputModule)
	{
		// Create the wheel list
        wheelList = new WheelList();
	}

	/**
	 * Adds wheel to drive object using open-loop control.
	 *
	 * @param posX			X-location of the wheel [in]
	 * @param posY			Y-location of the wheel [in]
	 * @param axisX			X-component of unit vector describing the wheel's
	 * axis of rotation (direction of this vector defines positive direction for
	 * wheel rotation according to right hand rule)
	 * @param axisY			Y-component of unit vector describing the wheel's
	 * axis of rotation (direction of this vector defines positive direction for
	 * wheel rotation according to right hand rule)
	 * @param rollerAngle	Angle between vector defining the axis of rotation
	 * and the rolling element in contact with the ground [deg]
	 * @param radius		Radius of the wheel [in]
	 * @param motorSlot		cRIO slot into which the digital sidecar sending the
	 * PWM signal to the motor controller is connected
	 * @param motorChannel	PWM channel on the digital sidecar to which the
	 * motor controller is connected
     * @param maxSpeed      Maximum speed achievable at this wheel [rad/sec]
	 */
    public void AddWheel(double posX, double posY, double axisX, double axisY,
                double rollerAngle, double radius,
                int motorSlot, int motorChannel, double maxSpeed)
    {
        // Create the wheel object and add it to the array
        Wheel newWheel = new Wheel(posX, posY, axisX, axisY, rollerAngle,
                radius, motorSlot, motorChannel, maxSpeed);

        wheelList.Add(newWheel);
    }

	public boolean Initialize()
	{
		if (wheelList.Size() < 4)
			return false;

		int i;

		// Create the limiter objects for each wheel
        wheelSpeedLimiter = new SecondOrderLimiter[wheelList.Size()];
        double wheelMaxAccel;// [rad/sec^2]
        for (i = 0; i < wheelList.Size(); i++)
        {
            wheelMaxAccel = friction * 12 * 32.174
                    / wheelList.Get(i).GetRadius();
            wheelSpeedLimiter[i] = new SecondOrderLimiter(1.0,
					wheelMaxAccel / wheelList.Get(i).GetMaxRotationRate(), freq);
        }

		return true;
	}

	public void Drive(Joystick stick)
	{
		// Joystick Y axis if fore-aft, positive aft (requires sign change)
		double yawCmd = ApplyDeadband(stick.getTwist());// CHECK THIS - does it need to be inverted? Also, could be changed to .getX() if preferred.
		double forwardCmd = ApplyDeadband(stick.getY());// CHECK THIS - does it need to be inverted?

		// Issue the velocity commands for each wheel
        int i;
		double wheelSpeedCmd;
        for (i = 0; i < wheelList.Size(); i++)
        {
			wheelSpeedCmd = forwardCmd;
			if (((Wheel)wheelList.Get(i)).GetRollerAngle() == 0.0)
			{
				if (((Wheel)wheelList.Get(i)).GetXPos() > 0.0)
					wheelSpeedCmd += yawCmd;
				else
					wheelSpeedCmd -= yawCmd;
			}

			wheelSpeedCmd *= ((Wheel)wheelList.Get(i)).GetRotationAxisX();
System.out.println(wheelSpeedCmd);
            // Apply velocity and acceleration limits to the wheel speed
           // wheelSpeedCmd = wheelSpeedLimiter[i].Process(wheelSpeedCmd);
                // System.out.println(i +" "+wheelSpeedCmd);
			((Wheel)wheelList.Get(i)).DoControl(wheelSpeedCmd
					* ((Wheel)wheelList.Get(i)).GetMaxRotationRate());
        }
	}

	 /**
	 * Sets the surface friction coefficient (creates an acceleration limit for
	 * the robot).
	 *
	 * @param mu Coefficient of friction [-]
	 */
    public void SetFrictionCoefficient(double mu)
    {
        friction = mu;
    }

	/**
	 * Sets a deadband for each joystick axis around the center position.
	 *
	 * @param _deadband	Value between 0 and 1 specifying the percent of stick
	 * travel (or twist) to set to zero [%]
	 *
	 * @throws IllegalArgumentException
	 */
	public void SetDeadband(double _deadband) throws IllegalArgumentException
	{
		if (_deadband < 0.0 || _deadband > 1.0)
			throw new IllegalArgumentException("Deadband must have value " +
					"between 0 and 1");

		deadband = _deadband;

		SetDBSlopeIntercept();
	}

	/**
	 * Sets a minimum output level for each motor (jump from zero to this level)
	 * For closed-loop systems, it is recommended that this be ZERO.
	 *
	 * @param _deadband	Value between 0 and 1 specifying the minimum voltage
	 * command [%]
	 *
	 * @throws IllegalArgumentException
	 */
	public void SetMinimumOutput(double _minOut) throws IllegalArgumentException
	{
		if (_minOut < 0.0 || _minOut > 1.0)
			throw new IllegalArgumentException("Deadband must have value " +
					"between 0 and 1");

		minimumOutput = _minOut;

		SetDBSlopeIntercept();
	}

	private void SetDBSlopeIntercept()
	{
		dbSlope = (1.0 - minimumOutput) / (1.0 - deadband);
		dbIntercept = 1.0 - dbSlope;
	}

	/**
	 * Returns the signal with the deadband applied.  Deadband is applied
	 * smoothly so that instead of a step at the edge of the deadband, the
	 * signal is interpolated linearly from the first "active" reading to the
	 * maximum joystick value.
	 *
	 * @param signal	Raw, unmodified signal [-1..1]
	 *
	 * @return Signal with deadband applied
	 */
	private double ApplyDeadband(double signal)
	{
		// Linearize the command so it is zero at +/- deadband, but changes
		// linearly to the extremes of travel (don't create a jump as we exit
		// the deadband)

		// This method is intended for use ONLY with variables varying from
		// -1.0 to 1.0

		if (Math.abs(signal) < deadband)
			return 0.0;

		/*if (signal > 0.0)
			return (signal - deadband) / (1.0 - deadband);
		else
			return (signal + deadband) / (1.0 - deadband);*/
		if (signal > 0.0)
			return dbSlope * signal + dbIntercept;
		else
			return dbSlope * signal - dbIntercept;
	}
}
