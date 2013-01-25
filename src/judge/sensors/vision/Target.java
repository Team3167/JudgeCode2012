/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package judge.sensors.vision;

// WPI imports
import edu.wpi.first.wpilibj.camera.*;
import edu.wpi.first.wpilibj.image.*;

// Judge imports
import judge.RobotConfiguration;

/**
 * Class representing the four net borders.
 * @author kloux
 */
public class Target
{
	/**
	 * Class for representing the portion of the image we are interested in.
	 */
	private static class Threshold
	{
		protected final int hue;
		protected final int saturation;
		protected final int luminance;

		public static final Threshold low = new Threshold(0, 0, 240);
		public static final Threshold high = new Threshold(240, 255, 255);

		/**
		 * Private constructor for Threshold class.
		 *
		 * @param h Hue, 0 to 255
		 * @param s Saturation, 0 to 255
		 * @param l Luminance, 0 to 255
		 */
		private Threshold (int h, int s, int l)
		{
			hue = h;
			saturation = s;
			luminance = l;
		}
	}

	/**
	 * Class for storing target location information.
	 */
	public static class Location
	{
		/**
		 * Constructor for Location class.
		 *
		 * @param _angle [rad], angle to rotate in order to be aligned
		 * @param _distance [ft], distance to target
		 * @param _height [ft], height of target center-of-mass above ground
		 */
		public Location (double _angle, double _distance, double _height)
		{
			angle = _angle;
			distance = _distance;
			height = _height;
		}

		public double angle;// [rad] positive nose left
		public double distance;// [ft] forward
		public double height;// [ft] above ground
	}

	private Location top = new Location(0.0, 0.0, 0.0);
	private Location bottom = new Location(0.0, 0.0, 0.0);
	private Location left = new Location(0.0, 0.0, 0.0);
	private Location right = new Location(0.0, 0.0, 0.0);
	private Location best;

	private static final AxisCamera camera = AxisCamera.getInstance();

	/**
	 * Constructor for the target class.
	 */
	public Target ()
	{
		// Set up the camera (NOTE: restart required when changing these)
        camera.writeRotation(AxisCamera.RotationT.k0);
        camera.writeResolution(AxisCamera.ResolutionT.k640x480);
        camera.writeCompression(30);
	}

	/**
	 * Returns the location of the top target, as seen by the camera
	 * @return
	 */
	public Location GetTopTarget()
	{
		if (top.distance > 0.0)
			return top;

		return new Location(0.0, -1.0, 0.0);
	}

	/**
	 * Returns the location of the bottom target, as seen by the camera
	 * @return
	 */
	public Location GetBottomTarget()
	{
		if (bottom.distance > 0.0)
			return top;

		return new Location(0.0, -1.0, 0.0);
	}

	/**
	 * Returns the location of the left target, as seen by the camera
	 * @return
	 */
	public Location GetLeftTarget()
	{
		if (left.distance > 0.0)
			return top;

		return new Location(0.0, -1.0, 0.0);
	}

	/**
	 * Returns the location of the right target, as seen by the camera
	 * @return
	 */
	public Location GetRightTarget()
	{
		if (right.distance > 0.0)
			return top;

		return new Location(0.0, -1.0, 0.0);
	}

	public Location GetBestTarget()
	{
		return best;
	}

	private static final double topThresholdHeight = 7.542;// [ft]
	private static final double bottomThresholdHeight = 4.625;// [ft]

	public void Update() throws NIVisionException, AxisCameraException
	{
		// Get the image from the camera, and remove the areas outside our
		// threshold specification
		ColorImage image = camera.getImage();
		BinaryImage binary = image.thresholdHSL(
                Threshold.low.hue, Threshold.high.hue,
                Threshold.low.saturation, Threshold.high.saturation,
                Threshold.low.luminance, Threshold.high.luminance);

		// Generate a particle analysis report for the image
		// Returns the four largest particles
		ParticleAnalysisReport[] colorHits = binary.getOrderedParticleAnalysisReports(4);
		image.free();
        binary.free();

		int i;
		Location temp;
		for (i = 0; i < colorHits.length; i++)
		{
			ParticleAnalysisReport report = colorHits[i];
			// FIXME:  Process the report to determine which target we're dealing with,
			// and then try to determine distance to the target (and our position on the field?)

			// For debugging:  print all the stats to the screen
			System.out.print(report.toString());

			if (true)// FIXME:  Some checking to make sure we do want to use this report
			{
				// Determine which target this is
				temp = GetBackboardLocation(report);
				if (temp.height > topThresholdHeight)
					top = temp;
				else if (temp.height < bottomThresholdHeight)
					bottom = temp;
				else if (report.center_mass_x_normalized < 0.0)
					left = temp;
				else
					right = temp;
			}
		}

		// Choose the best one
		// First, calculate the average
		double average = 0.0;
		int count = 0;
		if (top.distance > 0.0)
		{
			average = (average * count + top.distance) / (count + 1);
			count++;
			best = top;
		}
		if (bottom.distance > 0.0)
		{
			average = (average * count + bottom.distance) / (count + 1);
			count++;
			best = bottom;
		}
		if (left.distance > 0.0)
		{
			average = (average * count + left.distance) / (count + 1);
			count++;
			best = left;
		}
		if (right.distance > 0.0)
		{
			average = (average * count + right.distance) / (count + 1);
			count++;
			best = right;
		}

		// If there was only one target found (or none), we can stop here
		if (count < 2)
			return;

		// Recalculate the average, assuming that we found one less target
		// Choose the target with the minimum variation in distance when we
		// remove it from the set (weighted closest to average)
		double variation, tempAverage;
		double bestVariation = best.distance;
		if (top.distance > 0.0)
		{
			tempAverage = (average * count - top.distance) / (count - 1);
			variation = Math.abs(average - tempAverage);
			if (variation < bestVariation)
			{
				best = top;
				bestVariation = variation;
			}
		}
		if (bottom.distance > 0.0)
		{
			tempAverage = (average * count - bottom.distance) / (count - 1);
			variation = Math.abs(average - tempAverage);
			if (variation < bestVariation)
			{
				best = bottom;
				bestVariation = variation;
			}
		}
		if (left.distance > 0.0)
		{
			tempAverage = (average * count - left.distance) / (count - 1);
			variation = Math.abs(average - tempAverage);
			if (variation < bestVariation)
			{
				best = left;
				bestVariation = variation;
			}
		}
		if (right.distance > 0.0)
		{
			tempAverage = (average * count - right.distance) / (count - 1);
			variation = Math.abs(average - tempAverage);
			if (variation < bestVariation)
			{
				best = right;
				//bestVariation = variation;
			}
		}
	}

	private static Location GetBackboardLocation(ParticleAnalysisReport report)
	{
		double phi = com.sun.squawk.util.MathUtils.atan(
				report.boundingRectHeight / report.imageHeight
				* Math.tan(RobotConfiguration.cameraVFOV * 0.5));
		double distance = RobotConfiguration.targetHeight / 24.0 / Math.tan(phi);
		double angle = com.sun.squawk.util.MathUtils.asin(
				report.boundingRectWidth / RobotConfiguration.targetWidth);
		double height = RobotConfiguration.cameraHeight
				+ Math.tan(RobotConfiguration.cameraVFOV * 0.5)
				* report.center_mass_y_normalized * distance;

		return new Location(angle, distance, height);
	}
}
