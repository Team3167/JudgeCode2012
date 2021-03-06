/*******************************************************************************
* File:  RangeFinder20150.java
* Date:  4/6/2011
* Auth:  K. Loux
* Desc:  Class for IR rangefinder from www.sparkfun.com.  This is for the model
*        that senses 20 to 150 cm.
*******************************************************************************/

// Declare our package
package judge.sensors;

// WPI imports
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * Object for interfacing with an IR rangefinder (20 to 150 cm version).  Sensor
 * is available from www.sparkfun.com.
 *
 * @author K. Loux
 */
public class RangeFinder20150 extends SensorBase implements PIDSource
{
    // Fields
	private AnalogChannel analogChannel;

	// Methods
	/**
	 * Constructor.  Contains all necessary parameters to define the analog
	 * input channel.
	 *
	 * @param _slot		The slot on the cRIO in which the analog input module
	 * resides
	 * @param _channel	The channel on the specified analog input module that is
	 * wired to the sensor
	 */
	public RangeFinder20150(final int _slot, final int _channel)
	{
		// Setup the slot and channel in the super class
		analogChannel = new AnalogChannel(_slot, _channel);

		// Set up the oversampling and averaging for the analog channel
		analogChannel.setAverageBits(0);
		analogChannel.setOversampleBits(10);
	}

	/**
	 * Returns the distance measured by the sensor in inches.  Returns zero if
	 * the signal is out of range.
	 *
	 * @return Measured distance in inches
	 *
	 * @throws IllegalStateException
	 */
	public double GetDistance() throws IllegalStateException
	{
        // Get the voltage input
        double voltage = analogChannel.getAverageVoltage();

        // Check the range on the voltage - if it's outside of what we expect,
        // throw a sensor failure exception
        if (voltage < -0.5 || voltage > 3.75)
            throw new IllegalStateException("IR Rangefinder voltage out of "
					+ "range! (" + voltage + " V)");

		// If the voltage is between 0.5 and 2.75, return zero, because we can't
		// accurately determine distance, but don't throw an exception because
		// everything appears to be working OK
		if (voltage < 0.4 || voltage > 2.75)
			return 0.0;

		// Using a curve fit from the datasheet, convert this into distance
		return com.sun.squawk.util.MathUtils.pow(60.80397 * voltage,
                -1.17974) / 2.54;
	}
	
	/**
	 * Returns the measured value.
	 * @return the measured distance
	 */
	public double pidGet()
	{
		return GetDistance();
	}
}