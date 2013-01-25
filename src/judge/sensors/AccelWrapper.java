/*******************************************************************************
* File:  AccelWrapper.java
* Date:  1/10/2012
* Auth:  K. Loux
* Desc:  Wrapper for ADXL345_I2C accelerometer to make it a PID source.
*******************************************************************************/

// Declare our package
package judge.sensors;

// WPI imports
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * Wrapper for ADXL345_I2C accelerometer to make it a PID source.
 * 
 * @author kloux
 */
public class AccelWrapper implements PIDSource
{
	// The sensor object
	private ADXL345_I2C accelerometer;
	private ADXL345_I2C.Axes axis;
	
	/**
	 * Constructor for the I2C accelerometer wrapper
	 * 
	 * @param _accelerometer
	 * @param _axis 
	 */
	public AccelWrapper(ADXL345_I2C _accelerometer, ADXL345_I2C.Axes _axis)
	{
		accelerometer = _accelerometer;
		axis = _axis;
	}
	
	/**
	 * Returns the acceleration sensed in the specified axis.
	 * @return acceleration in Gs
	 */
	public double pidGet()
	{
		return accelerometer.getAcceleration(axis);
	}
}
