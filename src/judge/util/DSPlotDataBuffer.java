/*******************************************************************************
* File:  SDPlotDataBuffer.java
* Date:  1/26/2011
* Auth:  K. Loux
* Desc:  Provides a means for objects outside of the main robot class to add
*        data to the buffer to be sent for plotting on the driver station.
*******************************************************************************/

// Declare our package
package judge.util;

/**
 * Object for buffering data being passed to the Driver's Station for display on
 * the custom plot.
 *
 * @author K. Loux
 */
public class DSPlotDataBuffer
{
    // Fields
	private static final int curveCount = 8;
	private float data[] = new float[curveCount];

	// Methods
	/**
	 * Stores data in the buffer for the desired curve
	 *
	 * @param index Specifies the desired curve
	 * @param value	Value to store in the buffer
	 *
	 * @throws IllegalArgumentException
	 */
	public void SetData(int index, float value) throws IllegalArgumentException
	{
		if (index >= curveCount || index < 0)
			throw new IllegalArgumentException("Error:  Index out of bounds");

		data[index] = value;
	}

	/**
	 * Returns the data for the curve with the specified index
	 *
	 * @param index	Specifyies the desired curve
	 *
	 * @return Data stored for the specified curve
	 *
	 * @throws IllegalArgumentException
	 */
	public float GetData(int index) throws IllegalArgumentException
	{
		if (index >= curveCount || index < 0)
			throw new IllegalArgumentException("Error:  Index out of bounds");

		return data[index];
	}

	/**
	 * Returns the number of curves supported by this buffer.
	 *
	 * @return Size of the buffer in units of supported curves
	 */
	public int Size()
	{
		return curveCount;
	}
}