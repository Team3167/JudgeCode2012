/*******************************************************************************
* File:  Turret.java
* Date:  1/10/2012
* Auth:  K. Loux
* Desc:  Class for controlling a rotating, target-following turret.
*******************************************************************************/

// Declare our package
package judge.drive;

// WPI imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;

// Local imports
import judge.sensors.KalmanFilter;
import judge.util.PIDControllerII;
import judge.RobotConfiguration;
import judge.sensors.CameraWrapper;
import judge.math.Matrix;

/**
 * Class for controlling a rotating, target-following turret.
 *
 * @author kloux
 */
public class Turret
{
	private final double freq = RobotConfiguration.frequency;// [Hz]

	// Turret states
	private static final int stateManual = 0;
	private static final int stateSeeking = 1;
	private static final int stateTracking = 2;
	private int state = stateSeeking;

	// Control objects
	private KalmanFilter estimator;
	private Matrix uMatrix;
	private Matrix stateVector;
	private PIDControllerII positionLoop;
	private PIDMotor turretMotor;
	private PIDMotor pitchingMotor;
	private double positionError;// [deg]
	private double lastPositionError;// [deg]
	// NOTE:  Maybe some method to specify which basket to shoot at? Or not...

	private double pitchingSpeed;// [rad/sec]
	private final double maxPitchingRange = 30.0;// [ft] FIXME:  Make this calculation automatic?

	// Input devices
	private Joystick driverStick;
	private Joystick gunnerStick;
	private static final double maxTurretVelocity = 15.0 * Math.PI / 180.0;// [rad/sec]
	private static final double seekScale = 0.5;// [%]
	private CameraWrapper camera;
	private Encoder encoder;
	private Gyro gyro;

	public Turret(Joystick driver, Joystick gunner)
	{
		driverStick = driver;
		gunnerStick = gunner;

		// Initialize class members
		camera = new CameraWrapper();

		gyro = new Gyro(RobotConfiguration.analogInputModule,
				RobotConfiguration.yawGyroChannel);

		encoder = new Encoder(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.turretEncoderAChannel,
				RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.turretEncoderBChannel);// High-res by default
		encoder.setDistancePerPulse(
				RobotConfiguration.usrEncoderAnglePerPulse);// FIXME:  This isn't right (also needs gear ratio, etc.)
		encoder.start();

		estimator = new KalmanFilter();
		ConfigureKalmanFilter();
		stateVector = new Matrix(estimator.GetStateSize(), 1);

		uMatrix = new Matrix(3, 1);// FIXME:  These dimensions may change

		turretMotor = new PIDMotor(RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.turretMotorChannel,
				PIDMotor.Motor.motorFisherPrice, 1.0, 1.0,// FIXME:  Gear ratios need work here
				RobotConfiguration.turretVKp,
				RobotConfiguration.turretVKi,
				RobotConfiguration.turretVQueueSize,
				RobotConfiguration.turretVOmega,
				RobotConfiguration.turretVZeta,
				RobotConfiguration.frequency,
				RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.turretEncoderAChannel,
				RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.turretEncoderBChannel,
				RobotConfiguration.usrEncoderPulsesPerRevolution,
				false);

		positionLoop = new PIDControllerII(RobotConfiguration.turretPKp,
				RobotConfiguration.turretPKi,
				RobotConfiguration.turretPQueueSize, freq);
	}

	public void Update()
	{
		camera.Update();

		switch (state)
		{
			default:
			case stateManual:
				// Let the driver use the joystick to control the turret
				positionError =
						gunnerStick.getTwist() * maxTurretVelocity / freq;
				pitchingSpeed = GetPitchingSpeedFromDistance(
						gunnerStick.getThrottle() * maxPitchingRange);// FIXME:  Check range (0 to 1 or -1 to 1?)
				break;

			case stateSeeking:
				// Spin at some pre-determined speed, scanning for some geometry
				// we recognize
				if (lastPositionError > 0)
					positionError = seekScale * maxTurretVelocity / freq;
				else
					positionError = -seekScale * maxTurretVelocity / freq;

				pitchingSpeed = 0.0;// Do we really want to stop here?

				if (camera.TagetInView())
					state = stateTracking;
				break;

			case stateTracking:
				// Update our estimate of the angle to the target using the
				// Kalman filter
				uMatrix.SetElement(0, 0, driverStick.getY());// FIXME:  Make sure these match directions and signs
				uMatrix.SetElement(1, 0, driverStick.getX());
				uMatrix.SetElement(2, 0, driverStick.getTwist());
				stateVector = estimator.UpdateEstimate(uMatrix);
				positionError = stateVector.GetElement(0, 0);

				// Update position error so we can make a better decision about
				// which direction to spin to get back on target
				lastPositionError = positionError;

				pitchingSpeed = GetPitchingSpeedFromDistance(
						stateVector.GetElement(1, 0));

				if (!camera.TagetInView())
					state = stateSeeking;
				break;
		}

		// Close the PID loop (motor control inside velocity loop inside
		// position loop)
		turretMotor.DoControl(positionLoop.DoControl(positionError));

		// Set the pitching machine speed
		pitchingMotor.DoControl(pitchingSpeed);
	}

	/**
	 * Sets the turret to be in manual mode
	 */
	public void SetManualMode()
	{
		state = stateManual;
	}

	/**
	 * Sets the turret to be in automatic mode
	 */
	public void SetAutomaticMode()
	{
		state = stateSeeking;
	}

	/**
	 * Switches between manual and automatic modes
	 */
	public void ToggleMode()
	{
		if (state == stateManual)
			SetAutomaticMode();
		else
			SetManualMode();
	}

	/**
	 * Gets the name of the current state using length of 21 characters for LCD
	 * display.
	 *
	 * @return state name string
	 */
	public String GetStateName()
	{
		switch (state)
		{
			case stateManual:
				return "Turret Manual        ";

			case stateSeeking:
				return "Turret Auto-Seeking  ";

			case stateTracking:
				return "Turret Auto-Tracking ";

			default:
				return "Turret unknown state ";
		}
	}

	private void ConfigureKalmanFilter()
	{
		estimator.AddSensor(gyro);// Yaw rate of robot relative to world
		estimator.AddSensor(encoder);// Yaw rate of turret relative to robot
		estimator.AddSensor(camera.GetAngleSensor());// Horizontal angle of turret relative to target
		estimator.AddSensor(camera.GetDistanceSensor());// Distance to target

		// x is #x1:
		int xSize = 0;
		//    horizontal angle to target [rad]
		//    horizontal distance to target [ft]
		// u is #x1:
		int uSize = 0;
		//    FIXME:  Robot input commands?
		// z is 4x1:
		int zSize = 4;
		//    robot-fixed yaw gyro [deg]
		//    turret encoder [deg]
		//    horizontal angle to target (from camera) [deg]
		//    horizontal distance to target (from camera) [ft]
		Matrix h = new Matrix(zSize, xSize);
		Matrix r = new Matrix(zSize, zSize);
		Matrix q = new Matrix(xSize, xSize);
		Matrix a = new Matrix(xSize, xSize);
		Matrix b = new Matrix(xSize, uSize);
		Matrix x = new Matrix(xSize, 1);
		Matrix p = new Matrix(xSize, xSize);

		// Populate and assign the matrices
		h.MakeZero();
		/*h.SetElement(0, 0, -1.0);// accelerometer measures sine of theta (small angle approx.)
		h.SetElement(0, 4, 1.0 / RobotConfiguration.gravity);// accelerometer also affected by linear acceleration
		h.SetElement(1, 1, 1.0 * Math.PI / 180.0);// gyro measures theta (not theta dot)
		h.SetElement(2, 3, 1.0);// encoder measures robot velocity*/
		estimator.SetObservationMatrix(h);

		r.MakeZero();
		/*r.SetElement(0, 0, RobotConfiguration.accelerometerVariance);// accelerometer
		r.SetElement(1, 1, Math.PI * Math.PI / 180.0 / 180.0
				* RobotConfiguration.gyroRateVariance);// gyro
		r.SetElement(2, 2, 0.1);// encoder FIXME:  This will need to change with units*/
		estimator.SetMeasurementNoiseCovariance(r);

		// FIXME:  These need to be thought out better
		q.MakeZero();
		/*q.SetElement(1, 1, 0.5);
		q.SetElement(2, 2, 0.1);
		q.SetElement(3, 3, 0.0);
		q.SetElement(4, 4, 0.0);
		q.SetElement(5, 5, 0.0);*/
		estimator.SetProcessNoiseCovariance(q);

		/*x.SetElement(0, 0, 10.0 * Math.PI / 180.0);// 10 deg from horizontal FIXME:  What is real angle?
		x.SetElement(1, 0, 0.0);// zero rotational speed
		x.SetElement(2, 0, -5.0);// near low end of bridge (driver MUST go on forwards)
		x.SetElement(3, 0, 0.0);// actual robot speed FIXME:  Add this!
		x.SetElement(4, 0, 0.0);// zero acceleration*/
		estimator.SetStateVector(x);

		p.MakeZero();
		//p.SetElement(0, 0, 0.0);// very confident of bridge angle
		//p.SetElement(1, 1, 0.0);// very confident of bridge speed
		/*p.SetElement(2, 2, 1.0);// not very confident of robot position
		p.SetElement(3, 3, 0.1);// very confident of robot speed
		p.SetElement(4, 4, 5.0);// somewhat confident of robot acceleration*/
		estimator.SetStateCovarianceMatrix(p);

		a.MakeIdentity();
		/*a.SetElement(0, 1, RobotConfiguration.timeStep);
		a.SetElement(1, 2, RobotConfiguration.timeStep *
				RobotConfiguration.gravity * RobotConfiguration.robotMass *
				x.GetElement(2, 0) / bridgeInertia);
		a.SetElement(2, 3, RobotConfiguration.timeStep);
		a.SetElement(3, 4, RobotConfiguration.timeStep);*/
		estimator.SetStateTransitionMatrix(a);

		b.MakeZero();
		/*b.SetElement(4, 0, 1.0 * RobotConfiguration.frequency);
		b.SetElement(4, 1, -1.0 * RobotConfiguration.frequency);*/
		estimator.SetInputMatrix(b);

		// Finish filter initialization
		estimator.Initialize();

		// FIXME:  More kalman filter stuff required
		// We can use yaw input, but without knowing our orientation relative to
		// world, x and y translation commands aren't much help (maybe absolute encoder coupled with camera give this?)


	}

	private double GetPitchingSpeedFromDistance(double distance)
	{
		double launchAngle = 45.0 * Math.PI / 180.0;// [rad] FIXME:  Fixed?
		double deltaHeight = camera.GetTargetLocation().height
				- RobotConfiguration.cameraHeight;// [ft]
		double hangTime = Math.sqrt(2.0 / RobotConfiguration.gravity
				* (distance * Math.tan(launchAngle) - deltaHeight));

		return distance / hangTime / Math.cos(launchAngle);
	}
}
