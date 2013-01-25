/*******************************************************************************
* File:  Navigator.java
* Date:  1/17/2011
* Auth:  K. Loux
* Desc:  This class performs two main functions:  closes the robot position loop
*        and updating our position estimate.  This requires defining a field
*        coordinate system:
*            X: cross-field direction (short-ways or side-to-side) [in]
*                Zero At:    Field boundary to the left of the home operator's
*                            station (as viewed by the operator)
*                Pos. Dir.:  To the right (always positive while robot is in
*                            field of play)
*            Y: down-field direction (long-ways) [in]
*                Zero At:    Field boundary along home side (directly in front
*                            of the operators)
*                Pos. Dir.:  Down field (always positive while robot is in field
*                            of play)
*            Theta: rotation [deg]
*                Zero At:    Robot oriented along Y-axis (pointed down field)
*                Pos. Dir.:  Counter-clockwise rotation when viewed from above
*******************************************************************************/

// Declare our package
package judge.drive;

// libWPI imports
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.ADXL345_I2C;

// Local imports
import judge.math.Matrix;
import judge.sensors.RangeFinder1080;

/**
 * Class for estimating the robot's positon.  Gets input from the drive class
 * (wheel encoders), gyroscope, accelerometer, and can be set directly from
 * code that estimates the position using other sensors (IR rangefinders, line
 * sensors, etc.)
 *
 * @author K. Loux
 */
public class Navigator
{
    // Fields
    // Drive object
    private HolonomicRobotDrive drive;

    private double freq;// [Hz]
    private double lastTheta;// [deg]

    // Current position estimate (inches and degrees)
    private double x = 0.0, y = 0.0, theta = 0.0;
	private double xOffset, yOffset, thetaOffset;

	// Current velocity estimate
	private double vX, vY, omega;

	// Fields for updating the position estimate (from sensors)
	private Matrix velFromWheelSpeed;
	private double xScale = 0.67;// allow for slipping in the X direction

    // Accelerometer and gyro objects for estimating our position
    private Gyro gyro;
    private ADXL345_I2C accel;

	// Range sensor objects
	private RangeFinder1080 leftFront, rightFront, leftRear, rightRear;

	// Locations of the sensors
	private double gyroXPos, gyroYPos, gyroOrientation;
	private double accelXPos, accelYPos, accelOrientation;

    // Methods
    /**
	 * Constructor.
	 *
	 * @param _drive			Drive object from which estimates based on wheel
	 * encoders are retrieved
	 * @param _freq				Fixed frequency at which the positon estimate is
	 * updated [Hz]
	 * @param gyroSlot			cRIO slot into which the analog input module
	 * supporting the gyroscope is connected
	 * @param gyroChannel		Channel on the analog input module into which
	 * the gyroscope is connected
	 * @param accelSlot			cRIO slot into which the digital sidecar
	 * supporting the accelerometer I2C is connected
	 * @param _gyroXPos			The X position of the gyroscope on the robot
	 * [in]
	 * @param _gyroYPos			The Y position of the gyroscope on the robot
	 * [in]
	 * @param _gyroOrientation	The orientation of the gyroscope on the robot
	 * [deg]
	 * @param _accelXPos		The X position of the accelerometer on the robot
	 * [in]
	 * @param _accelYPos		The Y position of the accelerometer on the robot
	 * [in]
	 * @param _accelOrientation	The orientation of the accelerometer on the
	 * robot [deg]
	 * @param gyroSensitivity	Sensitivity of the gyroscope [V-sec/deg]
	 * @param accelRange		Measurement range of the accelerometer
	 */
    public Navigator(HolonomicRobotDrive _drive, double _freq,
            int gyroSlot, int gyroChannel, int accelSlot,
			double _gyroXPos, double _gyroYPos, double _gyroOrientation,
			double _accelXPos, double _accelYPos, double _accelOrientation,
			double gyroSensitivity, ADXL345_I2C.DataFormat_Range accelRange)
    {
        // Assing local fields
        drive = _drive;
        freq = _freq;

        // Create the accelerometer and gyro objects
        gyro = new Gyro(gyroSlot, gyroChannel);
        accel = new ADXL345_I2C(accelSlot, accelRange);

        // Initialize the gyroscope
        gyro.setSensitivity(gyroSensitivity);// [V-sec/deg]
		gyroXPos = _gyroXPos;
		gyroYPos = _gyroYPos;
		gyroOrientation = _gyroOrientation;

		// Set up the accelerometer (using I2C communication)
		accelXPos = _accelXPos;
		accelYPos = _accelYPos;
		accelOrientation = _accelOrientation;

        // Initialize offset variables
        thetaOffset = 0.0;
        xOffset = accel.getAcceleration(ADXL345_I2C.Axes.kX);
        yOffset = accel.getAcceleration(ADXL345_I2C.Axes.kY);
    }

    /**
	 * Sets the current robot position, in case position is determined from
	 * sensors outside of this class.
	 *
	 * @param _x		X position of the robot [in]
	 * @param _y		Y position of the robot [in]
	 * @param _theta	Orientation of the robot [deg]
	 */
    public void SetPosition(double _x, double _y, double _theta)
    {
        // Assign the values
        x = _x;
        y = _y;

		//System.out.println("old: " + theta + "  new: " + _theta);
		lastTheta -= theta - _theta;
        theta = _theta;
        thetaOffset = theta + gyro.getAngle();
    }

    // Method for updating the position estimate based on feedback from gyro,
    // accelerometers, and wheel encoders
	/**
	 * Updates the current position estimate.  MUST be called at the fixed
	 * frequency specified when constructed.  Position estimate is updated based
	 * on feedback from gyroscope, accelerometer, and wheel encocders
	 * (transformed from wheel speeds into robot speeds by HolonomicRobotDrive).
	 */
    public void Update()
    {
        // Blend the accel and gyro signals with the velocities estimated from
        // encoder feedback
        // This would be the right place to implement a Kalman filter

		// From reading on Wikipedia, I believe the following is possible:
		// Update the state estimate (based on control vector) and the estimate
		// covariance on each time step
		// Update the measurement residual whenever data is available for that
		// measurement (irregular intervals are OK) - this is every time step
		// for the gyro, accelerometers and wheel speed sensors, but it can be
		// when we are in range for the "rangefinder" sensors

		// Retrieve feedback from all of the sensors that always provide
		// feedback
		velFromWheelSpeed = drive.GetEstimatedRobotVelocity();

        double accelFactor = 0.0;
        double gyroFactor = 1.0;

		// FIXME:  use locations of acceleromters and gyro for more accurate measurements?
        // Get accelerometer readings and integrate to estimate velocity
        ADXL345_I2C.AllAxes accelerations = accel.getAccelerations();
        vX -= (accelerations.XAxis - xOffset) * 32.174 * 12.0 / freq;
        vY -= (accelerations.YAxis - yOffset) * 32.174 * 12.0 / freq;

        vX = velFromWheelSpeed.GetElement(0, 0) *
                (1.0 - accelFactor) + vX * accelFactor;
		vY = velFromWheelSpeed.GetElement(1, 0) *
                (1.0 - accelFactor) + vY * accelFactor * xScale;
        /*omega = (-gyro.getAngle() + thetaOffset - lastTheta) * freq * gyroFactor
				+ velFromWheelSpeed.GetElement(2, 0) * (1.0 - gyroFactor);*/
		//System.out.println("raw angle: " + -gyro.getAngle());
		theta = -gyro.getAngle() + thetaOffset;

        // Integrate to update the position estimate
		// For this we need theta in radians
		double thetaRadians = theta * Math.PI / 180.0;
		x += (vX * Math.cos(thetaRadians) - vY * Math.sin(thetaRadians)) / freq;
		y += (vY * Math.cos(thetaRadians) + vX * Math.sin(thetaRadians)) / freq;
		//theta += omega / freq;

		lastTheta = theta;

		//System.out.println(theta + "   " + omega);
		//System.out.println("vX: " + vX + "  vy: " + vY);
		//System.out.println("x: " + x + "   y: " + y);
    }

    /**
	 * Returns the current X position estimate.
	 *
	 * @return X position estimate [in]
	 */
    public double GetX()
    {
        return x;
    }

	/**
	 * Returns the current Y position estimate.
	 *
	 * @return Y position estimate [in]
	 */
    public double GetY()
    {
        return y;
    }

	/**
	 * Returns the current theta position estimate.
	 *
	 * @return Theta position estimate [deg]
	 */
    public double GetTheta()
    {
        return theta;
    }

	/**
	 * Returns the current omega velocity estimate.
	 *
	 * @return Omega velocity estimate [deg/sec]
	 */
    public double GetOmega()
    {
        return omega;
    }

	/**
	 * Creates optional IR sensor objects.
	 *
	 * @param slot				Digital sidecar slot into which the analog input
	 * module supporting these sensors is connected
	 * @param leftFrontChan		Channel on analog input module into which the
	 * left front channel is connected
	 * @param rightFrontChan	Channel on analog input module into which the
	 * right front channel is connected
	 * @param leftRearChan		Channel on analog input module into which the
	 * left rear channel is connected
	 * @param rightRearChan		Channel on analog input module into which the
	 * right rear channel is connected
	 */
	public void CreateIRSensors(int slot, int leftFrontChan, int rightFrontChan,
			int leftRearChan, int rightRearChan)
	{
		// Create all four sensor objects
		leftFront = new RangeFinder1080(slot, leftFrontChan);
		rightFront = new RangeFinder1080(slot, rightFrontChan);
		leftRear = new RangeFinder1080(slot, leftRearChan);
		rightRear = new RangeFinder1080(slot, rightRearChan);
	}

	/**
	 * Returns the reading from the left front IR distance sensor.
	 *
	 * @return Distance reading from left front IR sensor [in]
	 */
    public double GetLeftFront()
    {
		try
		{
			return leftFront.GetDistance();
		}
		catch (Exception ex)
		{
			System.err.println(ex.toString());
			return 0.0;
		}
    }

	/**
	 * Returns the reading from the right front IR distance sensor.
	 *
	 * @return Distance reading from right front IR sensor [in]
	 */
    public double GetRightFront()
    {
        try
		{
			return rightFront.GetDistance();
		}
		catch (Exception ex)
		{
			System.err.println(ex.toString());
			return 0.0;
		}
    }

	/**
	 * Returns the reading from the left rear IR distance sensor.
	 *
	 * @return Distance reading from left rear IR sensor [in]
	 */
    public double GetLeftRear()
    {
        try
		{
			return leftRear.GetDistance();
		}
		catch (Exception ex)
		{
			System.err.println(ex.toString());
			return 0.0;
		}
    }

	/**
	 * Returns the reading from the right rear IR distance sensor.
	 *
	 * @return Distance reading from right rear IR sensor [in]
	 */
    public double GetRightRear()
    {
        try
		{
			return rightRear.GetDistance();
		}
		catch (Exception ex)
		{
			System.err.println(ex.toString());
			return 0.0;
		}
    }
}