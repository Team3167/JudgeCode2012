/*******************************************************************************
* File:  BangBangController.java
* Date:  3/31/2011
* Auth:  K. Loux
* Desc:  Object for performing closed-loop bang-bang control.
*******************************************************************************/

// Declare our package
package judge.util;

// libWPI imports
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Relay;

/**
 * Class for closing a control loop using bang-bang control.  The control signal
 * output from this object has three states: 1, 0, or -1.
 *
 * @author K. Loux
 */
public class BangBangController
{
	// Fields
	private final Jaguar jag;
	private final Relay relay;

	private final double tolerance;
	private final boolean reverseCmd;

	// Methods
	/**
	 * Constructor for driving a Relay object.
	 *
	 * @param _relay		Specifies the relay object to which the control
	 * signal is applied
	 * @param _tolerance	Tolerance on the error signal (deadband) - control
	 * signal is zero if error magnitude is less than tolerance
	 * @param _reverseCmd	Flag indicating that the control signal should be
	 * reversed from the standard convention
	 */
	public BangBangController(Relay _relay, double _tolerance,
			boolean _reverseCmd)
	{
		relay = _relay;
		jag = null;

		tolerance = _tolerance;
		reverseCmd = _reverseCmd;
	}

	// Jaguar constructor
	/**
	 * Constructor for driving a Jaguar object.
	 *
	 * @param _jag			Jaguar object to which the control signal is applied
	 * @param _tolerance	Tolerance on the error signal (deadband) - control
	 * signal is zero if error magnitude is less than tolerance
	 * @param _reverseCmd	Flag indicating that the control signal should be
	 * reversed from the standard convention
	 */
	public BangBangController(Jaguar _jag, double _tolerance,
			boolean _reverseCmd)
	{
		jag = _jag;
		relay = null;

		tolerance = _tolerance;
		reverseCmd = _reverseCmd;
	}

	/**
	 * Method for closing the control loop.
	 *
	 * @param cmd	Reference signal (setpoint)
	 * @param act	Feedback signal (measured response)
	 */
	public void DoControl(double cmd, double act)
	{
		// Handle relay version first
		if (relay != null)
		{
			if (cmd - act > tolerance)
				relay.set(Relay.Value.kForward);
			else if (cmd - act < -tolerance)
				relay.set(Relay.Value.kReverse);
			else
				relay.set(Relay.Value.kOff);
		}
		else
		{
			if (cmd - act > tolerance)
			{
				if (reverseCmd)
					jag.set(-1.0);
				else
					jag.set(1.0);
			}
			else if (cmd - act < -tolerance)
			{
				if (reverseCmd)
					jag.set(1.0);
				else
					jag.set(-1.0);
			}
			else
				jag.set(0.0);
		}
	}
}