/*******************************************************************************
* File:  PneumaticCylinder.java
* Date:  3/18/2011
* Auth:  K. Loux
* Desc:  Object for handling pneumatic cylinders and the solenoid firing them.
*******************************************************************************/

// Declare our package
package judge.drive;

// libWPI imports
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Class representing pneumatic cylinders and the solenoids firing them.
 *
 * @author K. Loux
 */
public class PneumaticCylinder
{
	// Solenoid objects for retracting/extending
	private final Solenoid extendSolenoid;
	private final Solenoid retractSolenoid;

    private final int extendCount;
    private final int retractCount;

	private final int solenoidFireCountLimit;
	private int solenoidFireCount = 0;

	private boolean extend = false, retract = true;

	// Methods
	/**
	 * Constructor.
	 *
	 * @param solenoidSlot		cRIO slot into which the solenoid module is
	 * plugged
	 * @param extendChannel		Solenoid channel to be raised high to extend the
	 * arm - set to zero in the case of spring return solenoids set to "normally
	 * extended"
	 * @param retractChannel	Solenoid channel to be raised high to retract
	 * the arm - set to zero in the case of spring return solenoids set to
	 * "normally retracted"
     * @param extendTime       Time required for the cylinder to extend [sec]
     * @param retractTime      Time required for the cylinder to retract [sec]
	 * @param freq				Fixed frequency at which the state of this
	 * object is updated
	 *
	 * @throws IllegalArgumentException
	 */
	public PneumaticCylinder(int solenoidSlot, int extendChannel,
            int retractChannel, double extendTime, double retractTime,
            double freq) throws IllegalArgumentException
	{
		// Create the solenoids
		if (extendChannel != 0)
			extendSolenoid = new Solenoid(solenoidSlot, extendChannel);
		else
			extendSolenoid = null;

		if (retractChannel != 0)
			retractSolenoid = new Solenoid(solenoidSlot, retractChannel);
		else
			retractSolenoid = null;

        // Assign local fields
        extendCount = (int)(extendTime * freq);
        retractCount = (int)(retractTime * freq);

		// We have to have at least one solenoid though
		if (retractSolenoid == null && extendSolenoid == null)
			throw new IllegalArgumentException(
					"Pneumaitc cylinders must have at least one solenoid!");

		// Compute the required count
		double solenoidFireTime = 0.2;// [sec]
		solenoidFireCountLimit = (int)(solenoidFireTime * freq);
	}

	/**
	 * Sets the solenoid(s) to extend the cylinder.
	 */
	public void Extend()
	{
		if (!extend)
			solenoidFireCount = 0;

		extend = true;
		retract = false;
	}

	/**
	 * Sets the solenoid(s) to retract the cylinder.
	 */
	public void Retract()
	{
		if (!retract)
			solenoidFireCount = 0;

		extend = false;
		retract = true;
	}

	/**
	 * Method to be called at the frequency specified when constructed; sets the
	 * state of the solenoid(s) to extend/retract the cylinder as desired.
	 */
	public void Update()
	{
		if (extend)
		{
			// If the extend solenoid exists, fire it
			if (extendSolenoid != null)
			{
				// Handle solenoids differently depending on whether it's spring
				// return or dual-acting
				if (retractSolenoid != null
						&& solenoidFireCount < solenoidFireCountLimit)
					extendSolenoid.set(true);
				else if (retractSolenoid == null)
					extendSolenoid.set(true);
				else
					extendSolenoid.set(false);
			}
			else
				retractSolenoid.set(false);
			solenoidFireCount++;
		}
		else if (retract)
		{
			// If the extend solenoid exists, fire it
			if (retractSolenoid != null)
			{
				// Handle solenoids differently depending on whether it's spring
				// return or dual-acting
				if (extendSolenoid != null
						&& solenoidFireCount < solenoidFireCountLimit)
					retractSolenoid.set(true);
				else if (extendSolenoid == null)
					retractSolenoid.set(true);
				else
					retractSolenoid.set(false);
			}
			else
				extendSolenoid.set(false);
			solenoidFireCount++;
		}
	}

    /**
     * Returns true if the cylinder is commanded to extend and the time since
     * the command was issued is greater than the time expected for the cylinder
     * to extend (set in constructor).
     *
     * @return True if the cylinder is extended, false otherwise
     */
    public boolean IsExtended()
    {
        // If the fire count is greater than the extend count, and the command
        // is extend, return true
        if (solenoidFireCount > extendCount && extend)
            return true;

        return false;
    }

    /**
     * Returns true if the cylinder is commanded to retract and the time since
     * the command was issued is greater than the time expected for the cylinder
     * to retract (set in constructor).
     *
     * @return True if the cylinder is retracted, false otherwise
     */
    public boolean IsRetracted()
    {
        // If the fire count is greater than the retract count, and the command
        // is retract, return true
        if (solenoidFireCount > retractCount && retract)
            return true;
        
        return false;
    }
}
