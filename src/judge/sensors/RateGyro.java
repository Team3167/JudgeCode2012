/*******************************************************************************
* File:  RateGyro.java
* Date:  1/11/2012
* Auth:  K. Loux
* Desc:  Alternative for Gyro object to allow for use as rate sensor.  More-or-
*        less copied and adapted from the Gryo class.
*******************************************************************************/

// Declare our package
package judge.sensors;

// WPI imports
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.parsing.ISensor;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Timer;

/**
 * Wrapper for Gyro object to allow for use as rate sensor.
 * @author kloux
 */
public class RateGyro extends SensorBase implements PIDSource, ISensor
{
	static final int kOversampleBits = 10;
    static final int kAverageBits = 0;
    static final double kSamplesPerSecond = 50.0;
    static final double kCalibrationSampleTime = 5.0;
    static final double kDefaultVoltsPerDegreePerSecond = 0.007;
    AnalogChannel m_analog;
    double m_voltsPerDegreePerSecond;
    double m_offset;// [V]
    boolean m_channelAllocated;
	//AccumulatorResult result;

    /**
     * Initialize the gyro.
     * Calibrate the gyro by running for a number of samples and computing the center value for this
     * part.
     * It's important to make sure that the robot is not moving while the centering calculations are
     * in progress, this is typically done when the robot is first turned on while it's sitting at
     * rest before the competition starts.
     */
    private void initGyro()
	{
        if (m_analog == null)
		{
            System.out.println("Null m_analog");
        }
        m_voltsPerDegreePerSecond = kDefaultVoltsPerDegreePerSecond;
        m_analog.setAverageBits(kAverageBits);
        m_analog.setOversampleBits(kOversampleBits);
        double sampleRate = kSamplesPerSecond * (1 << (kAverageBits + kOversampleBits));
        m_analog.getModule().setSampleRate(sampleRate);

        Timer.delay(1.0);
        m_analog.initAccumulator();

        Timer.delay(kCalibrationSampleTime);

        m_offset = m_analog.getAverageVoltage();
    }

    /**
     * Gyro constructor given a slot and a channel.
    .
     * @param slot The cRIO slot for the analog module the gyro is connected to.
     * @param channel The analog channel the gyro is connected to.
     */
    public RateGyro(int slot, int channel)
	{
        m_analog = new AnalogChannel(slot, channel);
        m_channelAllocated = true;
        initGyro();
    }

    /**
     * Gyro constructor with only a channel.
     *
     * Use the default analog module slot.
     *
     * @param channel The analog channel the gyro is connected to.
     */
    public RateGyro(int channel)
	{
        m_analog = new AnalogChannel(channel);
        m_channelAllocated = true;
        initGyro();
    }

    /**
     * Gyro constructor with a precreated analog channel object.
     * Use this constructor when the analog channel needs to be shared. There
     * is no reference counting when an AnalogChannel is passed to the gyro.
     * @param channel The AnalogChannel object that the gyro is connected to.
     */
    public RateGyro(AnalogChannel channel)
	{
        m_analog = channel;
        if (m_analog == null)
		{
            System.err.println("Analog channel supplied to Gyro constructor is null");
        }
		else
		{
            m_channelAllocated = false;
            initGyro();
        }
	}

    /**
     * Delete (free) the accumulator and the analog components used for the gyro.
     */
    public void free()
	{
        if (m_analog != null && m_channelAllocated)
		{
            m_analog.free();
        }
        m_analog = null;
    }

    /**
     * Return the actual rate in degrees per second.
     *
     * The rate is based on the current value corrected by the oversampling rate, the
     * gyro type and the A/D calibration values.
     * @return the current sensed rate in degrees/second.
     */
    public double getRate()
	{
        if (m_analog == null)
		{
            return 0.0;
        }
		else
		{
            return (m_analog.getVoltage() - m_offset) / m_voltsPerDegreePerSecond;
        }
    }

    /**
     * Set the gyro type based on the sensitivity.
     * This takes the number of volts/degree/second sensitivity of the gyro and uses it in subsequent
     * calculations to allow the code to work with multiple gyros.
     *
     * @param voltsPerDegreePerSecond The type of gyro specified as the voltage that represents one degree/second.
     */
    public void setSensitivity(double voltsPerDegreePerSecond)
	{
        m_voltsPerDegreePerSecond = voltsPerDegreePerSecond;
    }

    /**
     * Get the rate of the gyro for use with PIDControllers
     * @return the current rate according to the gyro
     */
    public double pidGet()
	{
        return getRate();
    }
}
