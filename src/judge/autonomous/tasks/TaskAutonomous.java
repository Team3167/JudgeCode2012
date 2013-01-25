/*******************************************************************************
* File:  TaskAutonomous.java
* Date:  3/1/2012
* Auth:  K. Loux
* Desc:  Task for doing *something* in autonomous mode.
*******************************************************************************/

// Declare our package
package judge.autonomous.tasks;

// Local imports
import judge.RobotConfiguration;
import judge.drive.Bucket;
import judge.drive.Pitcher;

/**
 *
 * @author kloux
 */
public class TaskAutonomous extends TaskBase
{
	private final Pitcher pitcher;
	private final Bucket bucket;

	private final double shortDistance = 22.0;// [ft]
	private final double mediumDistance = 23.0;// [ft]
	private final double longDistance = 24.0;// [ft]

	public class Distance
	{
		public final static byte distanceShort = 0;
		public final static byte distanceMedium = 1;
		public final static byte distanceLong = 2;
	}
	private final byte distance;

	// Timing variables
	private final double shooterWaitTime = 2.0;// [sec]
	private final double dumpTime = 2.0;// [sec]
	private double timer;// [sec]

	// States
	private static final byte stateWaitForShooter = 0;
	private static final byte stateLaunchFirst = 1;
	private static final byte stateDumpSecond = 2;
	private static final byte stateLaunchSecond = 3;

	public TaskAutonomous(Pitcher _pitcher, Bucket _bucket, byte _distance)
	{
		super("Autonomous Tasks     ");

		pitcher = _pitcher;
		bucket = _bucket;
		distance = _distance;

		SetNextState(stateWaitForShooter);
	}

	protected void EnterState()
	{
		switch (GetState())
		{
			case stateWaitForShooter:
			case stateDumpSecond:
				timer = 0.0;
				break;

			case stateLaunchFirst:
			case stateLaunchSecond:
				pitcher.Shoot();
				break;
		}
	}

	protected void ProcessState()
	{
		// Regardless of state
		switch (distance)
		{
			case Distance.distanceShort:
				pitcher.Set(shortDistance);
				break;

			case Distance.distanceMedium:
				pitcher.Set(mediumDistance);
				break;

			case Distance.distanceLong:
				pitcher.Set(longDistance);
				break;
		}

		switch (GetState())
		{
			case stateWaitForShooter:
				timer += RobotConfiguration.timeStep;
				if (timer > shooterWaitTime)
					SetNextState(stateLaunchFirst);
				break;

			case stateLaunchFirst:
				if (!pitcher.IsShooting())
					SetNextState(stateDumpSecond);
				break;

			case stateDumpSecond:
				bucket.Set(Bucket.Move.up);
				if (bucket.IsUp())
				{
					timer += RobotConfiguration.timeStep;
					if (timer > dumpTime)
						SetNextState(stateLaunchSecond);
				}
				break;

			case stateLaunchSecond:
				if (bucket.IsUp())
					bucket.Set(Bucket.Move.down);
				break;
		}
	}

	/**
	 * Determines if we've finished with the bridge
	 * @return
	 */
	protected boolean IsComplete()
	{
		if (GetState() == stateLaunchSecond && !pitcher.IsShooting())
			return true;

		return false;
	}

	/**
	 * Nothing required for this task
	 */
    protected void OnRemoveTask()
	{
	}

	/**
	 * Gets the name of the current state, processed for LCD output
	 *
	 * @return state name
	 */
	public String GetStateName()
	{
		switch (GetState())
		{
			case stateWaitForShooter:
				return "Waiting for shooter  ";

			case stateLaunchFirst:
				return "Launching first ball ";

			case stateDumpSecond:
				return "Dumping second ball  ";

			case stateLaunchSecond:
				return "Launching second ball";

			default:
				return "Unknown state        ";
		}
	}

	/**
	 * Checks to see if we should allow the driver to control the robot.
	 *
	 * @return
	 */
	protected boolean OkToDrive()
	{
		return true;
	}
}
