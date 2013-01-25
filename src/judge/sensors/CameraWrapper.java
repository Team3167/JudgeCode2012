/*******************************************************************************
* File:  CameraWrapper.java
* Date:  1/10/2012
* Auth:  K. Loux
* Desc:  PIDSource wrapper around the camera.  This is where targeting stuff
*        is programmed.
*******************************************************************************/

// Declare our package
package judge.sensors;

// WPI imports
import edu.wpi.first.wpilibj.PIDSource;

// Judge imports
import judge.sensors.vision.Target;

/**
 * PIDSource wrapper around the camera.  This is where targeting stuff
 * is programmed.
 * 
 * @author kloux
 */
public class CameraWrapper
{
	/**
	 * Dummy class serving as PIDSource for distance [ft]
	 */
	public class DistanceSensor implements PIDSource
	{
		private double value = 0;
		
		/**
		 * Returns current value
		 * 
		 * @return 
		 */
		public double pidGet()
		{
			return value;
		}
		
		/**
		 * Sets current value
		 * 
		 * @param v 
		 */
		public void Set(double v)
		{
			value = v;
		}
	}
	
	/**
	 * Dummy class serving as PIDSource for angle [deg]
	 */
	public class AngleSensor implements PIDSource
	{
		private double value = 0;
		
		/**
		 * Returns current value
		 * 
		 * @return 
		 */
		public double pidGet()
		{
			return value;
		}
		
		/**
		 * Sets current value
		 * 
		 * @param v 
		 */
		public void Set(double v)
		{
			value = v;
		}
	}
	
	// Objects for interface with Kalman filter
	private DistanceSensor distance;
	private AngleSensor angle;
	
	// Targets
	private Target target;
	private boolean targetInView = false;
	
	/**
	 * Constructor for the CameraWrapper class.
	 */
	public CameraWrapper()
	{
		distance = new DistanceSensor();
		angle = new AngleSensor();
		
		target = new Target();
	}
	
	/**
	 * Processes the image and updates appropriate values
	 */
	public void Update()
	{
		try
		{
			target.Update();
			
			Target.Location location = target.GetBestTarget();
			
			if (location.distance > 0.0)
				targetInView = true;
			else
				targetInView = false;

			distance.Set(location.distance);
			angle.Set(location.angle);
		}
		catch (Exception ex)
		{
			System.err.println(ex.toString());
			
			targetInView = false;
			distance.Set(0.0);
			angle.Set(0.0);
		}
	}
	
	/**
	 * Asks the camera if it sees the target
	 * @return 
	 */
	public boolean TagetInView()
	{
		return targetInView;
	}
	
	/**
	 * Returns a reference to the distance sensor object
	 * 
	 * @return distance to target (PIDSource)
	 */
	public DistanceSensor GetDistanceSensor()
	{
		return distance;
	}
	
	/**
	 * Returns a reference to the angle sensor object
	 * 
	 * @return horizontal angle to target (PIDSource)
	 */
	public AngleSensor GetAngleSensor()
	{
		return angle;
	}
	
	/**
	 * Returns the location of the best fitting target
	 * 
	 * @return 
	 */
	public Target.Location GetTargetLocation()
	{
		return target.GetBestTarget();
	}
}
