/*******************************************************************************
* File:  StateMachineBase.java
* Date:  4/4/2011
* Auth:  K. Loux
* Desc:  Base class for creating state machines.
*******************************************************************************/

// Declare our package
package judge.util;

/**
 * Base class for creating state machines.  Handles state transitions and
 * provides the option of overriding state entry, processing and exit methods.
 *
 * @author K. Loux
 */
public abstract class StateMachineBase
{
    private byte state = -1;// Initialize to something different than nextState
    private byte nextState = 0;// Assume initial state has value zero

    // Methods
    /**
     * Performs actions required on entry to current state.
     */
    protected void EnterState()
    {
        // Nothing required by default
    }

    /**
     * Handles normal state actions (not transitions).
     */
    protected void ProcessState()
    {
        // Nothing required by default
    }

    /**
     * Performs actions required on exit of current state.
     */
    protected void ExitState()
    {
        // Nothing required by default
    }

	/**
	 * Main method to be called from outside objects, automatically handles
	 * state transitions and calls the ProcessState() method for the current
	 * state.
	 */
    public final void Process()
    {
        // Check the current state against the next state to see if the state
        // changed
        if (state != nextState)
        {
            ExitState();
            state = nextState;
            EnterState();
        }

        // Do normal processing of the current state
        ProcessState();
    }

	/**
	 * Returns the byte representing the current state.
	 *
	 * @return Current state
	 */
	protected final byte GetState()
	{
		return state;
	}

	/**
	 * Sets the next state for the state machine (takes effect on next call of
	 * Proces()).
	 *
	 * @param _nextState Machine's next state
	 */
	protected final void SetNextState(byte _nextState)
	{
		nextState = _nextState;
	}
}