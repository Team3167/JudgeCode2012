/*******************************************************************************
* File:  BalanceFilter.java
* Date:  1/10/2012
* Auth:  K. Loux
* Desc:  Kalman filter implementation for estimating state of robot balancing on
*        teetering platform.
*******************************************************************************/

// Declare our package
package judge;

// WPI imports
import edu.wpi.first.wpilibj.Encoder;

// Local imports
import judge.sensors.KalmanFilter;
import judge.sensors.AccelWrapper;
import judge.sensors.RateGyro;
import judge.math.Matrix;

/**
 * Kalman filter implementation for estimating state of robot balancing on
 *        teetering platform.
 * @author kloux
 */
public class BalanceFilter extends KalmanFilter
{
	/**
	 * Constructor for balance filter
	 * 
	 * @param initialState
	 * @param initialCovariance 
	 */
	public BalanceFilter(AccelWrapper accel, RateGyro gyro, Encoder encoder)
	{
		// Make sure the encoder is set to give us velocity
		encoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kRate);
		
		// Add the sensors
		AddSensor(accel);
		AddSensor(gyro);
		AddSensor(encoder);
		
		// Create the required matrices
		// x is 5x1: (really our system is the bridge with the robot on it)
		int xSize = 5;
		//    bridge angle to horizontal [rad]
		//    bridge rotational speed [rad/sec]
		//    robot position on bridge relative to CG over center [ft]
		//    robot velocity [ft/sec]
		//    robot acceleration [ft/sec2]
		//
		// u is 2x1:
		int uSize = 2;
		//    fore-aft speed command [ft/sec]
		//    last fore-aft speed command [ft/sec]
		//
		// z is 3x1:
		int zSize = 3;
		//    longitudinal accelerometer [G]
		//    pitch gyro [deg/sec]
		//    wheel speed encoder [???/sec] FIXME:  Check units
		Matrix h = new Matrix(zSize, xSize);
		Matrix r = new Matrix(zSize, zSize);
		Matrix q = new Matrix(xSize, xSize);
		Matrix a = new Matrix(xSize, xSize);
		Matrix b = new Matrix(xSize, uSize);
		Matrix x = new Matrix(xSize, 1);
		Matrix p = new Matrix(xSize, xSize);
		
		// Populate and assign the matrices
		h.MakeZero();
		h.SetElement(0, 0, -1.0);// accelerometer measures sine of theta (small angle approx.)
		h.SetElement(0, 4, 1.0 / RobotConfiguration.gravity);// accelerometer also affected by linear acceleration
		h.SetElement(1, 1, 1.0 * Math.PI / 180.0);// gyro measures theta (not theta dot)
		h.SetElement(2, 3, 1.0);// encoder measures robot velocity
		SetObservationMatrix(h);

		r.MakeZero();
		r.SetElement(0, 0, RobotConfiguration.accelerometerVariance);// accelerometer
		r.SetElement(1, 1, Math.PI * Math.PI / 180.0 / 180.0
				* RobotConfiguration.gyroRateVariance);// gyro
		r.SetElement(2, 2, 0.1);// encoder FIXME:  This will need to change with units
		SetMeasurementNoiseCovariance(r);

		// FIXME:  These need to be thought out better
		q.MakeZero();
		q.SetElement(2, 2, 0.1);// Allow for some variance in modelled omega
		q.SetElement(5, 5, 0.1);// Allow for some variance in modelled acceleration
		SetProcessNoiseCovariance(q);

		x.SetElement(0, 0, 10.0 * Math.PI / 180.0);// 10 deg from horizontal FIXME:  What is real angle?
		x.SetElement(1, 0, 0.0);// zero rotational speed
		x.SetElement(2, 0, -5.0);// near low end of bridge (driver MUST go on forwards)
		x.SetElement(3, 0, 0.0);// actual robot speed FIXME:  Add this!
		x.SetElement(4, 0, 0.0);// zero acceleration
		SetStateVector(x);

		p.MakeZero();
		//p.SetElement(0, 0, 0.0);// very confident of bridge angle
		//p.SetElement(1, 1, 0.0);// very confident of bridge speed
		p.SetElement(2, 2, 1.0);// not very confident of robot position
		p.SetElement(3, 3, 0.1);// very confident of robot speed
		p.SetElement(4, 4, 5.0);// somewhat confident of robot acceleration
		SetStateCovarianceMatrix(p);
		
		double bridgeInertia = 1.0;// [slug-ft2] (includes nearly-centered robot) FIXME: better value needed
		a.MakeIdentity();
		a.SetElement(0, 1, RobotConfiguration.timeStep);
		a.SetElement(1, 2, RobotConfiguration.timeStep *
				RobotConfiguration.gravity * RobotConfiguration.robotMass *
				x.GetElement(2, 0) / bridgeInertia);
		a.SetElement(2, 3, RobotConfiguration.timeStep);
		a.SetElement(3, 4, RobotConfiguration.timeStep);
		a.SetElement(4, 4, 0.0);// Acceleration has no memory
		SetStateTransitionMatrix(a);
		
		b.MakeZero();
		b.SetElement(4, 0, 1.0 * RobotConfiguration.frequency);
		b.SetElement(4, 1, -1.0 * RobotConfiguration.frequency);
		SetInputMatrix(b);

		// Finish filter initialization
		Initialize();
	}
}
