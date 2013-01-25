/*******************************************************************************
* File:  JooystickButton.java
* Date:  2/10/2011
* Auth:  K. Loux
* Desc:  Wrapper class to handle detecting a button press but only responding to
*        to each press once.  Method is also available for checking if button is
*        held continuously.  Supports one or two joysticks per button.
*******************************************************************************/

// Declare our package
package judge.util;

// WPI imports
import edu.wpi.first.wpilibj.Joystick;

/**
 * Class for handling joystick button presses.  Without special handling, button
 * presses may result in unexpected behaviour due to repeated processing (cRIO
 * runs at fast rate - calling methonds once per click requires this object or
 * similar code).
 *
 * @author K. Loux
 */
public class JoystickButton
{
    // Fields
	private boolean stateRequested = false;
	private final Joystick stick;
	private final Joystick alternateStick;
	private final int button;

	// Methods
	/**
	 * Constructor for buttons located on only one joystick.
	 *
	 * @param _stick	Joystick object from which button is read
	 * @param _button	Button number (which button on the joystick)
	 */
	public JoystickButton(Joystick _stick, int _button)
	{
		// Assign the joystick button we are checking to the local fields
		stick = _stick;
		button = _button;

		// No alternate stick
		alternateStick = null;
	}

	/**
	 * Constructor for buttons located on two joysticks.
	 *
	 * @param stick1	First stick from which button is read
	 * @param stick2	Second stick from which button is read
	 * @param _button	Button number (which button on the joysticks)
	 */
	public JoystickButton(Joystick stick1, Joystick stick2, int _button)
	{
		// Assign the joystick button we are checking to the local fields
		stick = stick1;
		alternateStick = stick2;
		button = _button;
	}

    /**
	 * Returns true only once per click of the button.  Will not return true
	 * until the button (or buttons) has (have) been released and re-clicked.
	 *
	 * @return True if this is the first time we're requesting the state of the
	 * button and it is in fact pressed, false otherwise
	 */
	public boolean HasJustBeenPressed()
	{
		// Handle dual joysticks
		if (alternateStick != null)
		{
			// If the button is pressed, and we havne't asked for the state yet,
			// return true and reset the stateRequested flag
			if ((stick.getRawButton(button) ||
					alternateStick.getRawButton(button)) && !stateRequested)
			{
				stateRequested = true;
				return true;
			}
			else if (!stick.getRawButton(button) &&
					!alternateStick.getRawButton(button))
				stateRequested = false;

			return false;
		}

		// Handle single joysticks
		// If the button is pressed, and we havne't asked for the state yet,
		// return true and reset the stateRequested flag
		if (stick.getRawButton(button) && !stateRequested)
		{
			stateRequested = true;
			return true;
		}
		else if (!stick.getRawButton(button))
			stateRequested = false;

		return false;
	}

    // Method returns true as long as button is held down
	/**
	 * Returns true as long as the button is being pressed.  Equivalent to
	 * Joystick.getRawButton() method in libWPI.
	 *
	 * @return True if the button is pressed, false otherwise
	 */
    public boolean IsPressed()
    {
		// Handle two joysticks
		if (alternateStick != null)
			return stick.getRawButton(button) ||
					alternateStick.getRawButton(button);

		// Handle single joysticks
		return stick.getRawButton(button);
    }
}