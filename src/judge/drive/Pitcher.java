/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

// Declare our package
package judge.drive;

// libWPI imports
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

// Local imports
import judge.util.PIDControllerII;
import judge.RobotConfiguration;

/**
 * Class representing the pitching machine "shooter" that fires the balls.
 *
 * @author kloux
 */
public class Pitcher
{
	/*private PIDMotor topMotor;
	private PIDMotor bottomMotor;*/
	private Jaguar topMotor;
	private Jaguar bottomMotor;

	private Servo ballRelease;

	private final double servoStopPosition = 0.20;// [-]
	private final double servoReleasePosition = .85;// [-]

	private double motorCmd;// [-]
	private double backSpin = 0.75;
	private final double releaseTime = 0.5;// [sec]
	private double shootTimer = releaseTime + 1.0;// [sec]

	// Shooter speed calcs
	private final double launchHeight = 30.4 / 12.0;// [in]
	private final double targetHeight = 98.0 / 12.0;// [in]
	private final double deltaHeight = targetHeight - launchHeight;// [in]

	/**
	 * Constructor for Pitcher class.  Creates objects on specified ports.
	 */
	public Pitcher()
	{
		/*topMotor = new PIDMotor(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.pitcherTopMotorChannel,
				PIDMotor.Motor.motorBaneBotsRS550,
				RobotConfiguration.pitcherGearboxRatio,
				RobotConfiguration.pitcherGearboxRatio,
				RobotConfiguration.pitcherKp,
				RobotConfiguration.pitcherKi,
				RobotConfiguration.pitcherQueueSize,
				RobotConfiguration.pitcherOmega,
				RobotConfiguration.pitcherZeta,
				RobotConfiguration.frequency,
				RobotConfiguration.digitalSideCarSlot,
				RobotConfiguration.pitcherTopEncoderAChannel,
				RobotConfiguration.digitalSideCarSlot,
				RobotConfiguration.pitcherTopEncoderBChannel,
				RobotConfiguration.usrEncoderPulsesPerRevolution,
				false);

		bottomMotor = new PIDMotor(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.pitcherBottomMotorChannel,
				PIDMotor.Motor.motorBaneBotsRS550,
				RobotConfiguration.pitcherGearboxRatio,
				RobotConfiguration.pitcherGearboxRatio,
				RobotConfiguration.pitcherKp,
				RobotConfiguration.pitcherKi,
				RobotConfiguration.pitcherQueueSize,
				RobotConfiguration.pitcherOmega,
				RobotConfiguration.pitcherZeta,
				RobotConfiguration.frequency,
				RobotConfiguration.digitalSideCarSlot,
				RobotConfiguration.pitcherBottomEncoderAChannel,
				RobotConfiguration.digitalSideCarSlot,
				RobotConfiguration.pitcherBottomEncoderBChannel,
				RobotConfiguration.usrEncoderPulsesPerRevolution,
				false);*/

		topMotor = new Jaguar(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.pitcherTopMotorChannel);
		bottomMotor = new Jaguar(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.pitcherBottomMotorChannel);

		ballRelease = new Servo(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.gateServoChannel);
	}

	/**
	 * Sets the motor speed for the specified distance.
	 *
	 * @param distance distance to the target in feet
	 */
	public void Set(double distance)
	{
		if (distance == 0.0)
		{
			motorCmd = 0.0;
			return;
		}

		// Calculate wheel speed from distance
		/*double hangTime = Math.sqrt(
				(distance * Math.tan(RobotConfiguration.launchAngle)
				- deltaHeight) / RobotConfiguration.gravity * 2.0);// [sec]
		double desiredBallSpeed = distance / hangTime
				/ Math.cos(RobotConfiguration.launchAngle);// [ft/sec]
		double desiredFinalWheelSpeed = desiredBallSpeed /
				RobotConfiguration.pitchingWheelDiameter * 2.0;// [rad/sec]
		double finalEnergy = 0.5 *
				(RobotConfiguration.pitcherSystemInertia
				* desiredFinalWheelSpeed * desiredFinalWheelSpeed
				+ RobotConfiguration.ballMass * desiredBallSpeed
				* desiredBallSpeed);// [ft-lbf]
		double initialWheelSpeed = Math.sqrt(finalEnergy * 2.0
				/ RobotConfiguration.pitcherSystemInertia);// [rad/sec]

		double adjustmentFactor = 0.95;// [-] (includes friction, etc.)
		motorCmd = initialWheelSpeed * 30.0 / Math.PI
				/ RobotConfiguration.baneBotsRS550MaxSpeed
				* RobotConfiguration.pitcherGearboxRatio / adjustmentFactor;*/
		motorCmd = (distance - 5.0) / 45.0 * 0.75 + 0.1;

		if (motorCmd < 0.0)
			motorCmd = 0.0;
		else if (motorCmd > 1.0)
			motorCmd = 1.0;
	}

	/**
	 * Resets the timer controlling ball release levers.
	 */
	public void Shoot()
	{
		// Don't allow shooting if the wheels aren't spinning
		if (motorCmd > 0.0)
			shootTimer = 0.0;
	}

	public void setBackSpin(double value)
	{
		backSpin = value;
	}

	public boolean IsShooting()
	{
		return shootTimer < releaseTime;
	}

	/**
	 * Method to be called at periodic frequency.  Ensures pitcher responds to
	 * commands issued by driver/autonomous profile.
	 */
	public void Update()
	{
		// Create some backspin by spinning the top motor a hair slower
		topMotor.set(-motorCmd * backSpin);
		bottomMotor.set(motorCmd);
		/*topMotor.DoControl(-motorCmd * 0.95);
		bottomMotor.DoControl(motorCmd);*/

		// Handle the release mechanism
		if (shootTimer < releaseTime)
		{
			ballRelease.set(servoReleasePosition);
			shootTimer += RobotConfiguration.timeStep;
		}
		else
			ballRelease.set(servoStopPosition);
	}

	public double GetSpeed()
	{
		return motorCmd;
	}
}