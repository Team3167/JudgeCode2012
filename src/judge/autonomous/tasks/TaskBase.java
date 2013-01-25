/*******************************************************************************
* File:  TaskBase.java
* Date:  1/13/2011
* Auth:  K. Loux
* Desc:  Abstract base class for task objects for use in autonomous mode.
*******************************************************************************/

// Declare our package
package judge.autonomous.tasks;

// Local imports
import judge.util.StateMachineBase;

/**
 * Abstract base class for creating robot tasks.
 *
 * @author K. Loux
 */
public abstract class TaskBase extends StateMachineBase
{
    // Fields
	private final String taskName;
    private boolean taskStarted = false;

	// Methods
	/**
	 * Constructor.  Assigns task name.  Must be called with super() in derived
	 * classes.
	 *
	 * @param _taskName	String describing this task (21 characters or less)
	 */
    protected TaskBase(String _taskName)
    {
        taskName = _taskName;
    }

	/**
	 * Calls Process() method from StateMachineBase (in derived classes) and
	 * sets a flag indicating that we've started performing this task.
	 */
    protected final void PerformTaskActions()
    {
        taskStarted = true;

		// Call the main state machine method that handles state transitions and
		// the main state handling methods (override at least ProcessState in
		// order to make this do something)
        Process();
    }

	/**
	 * Returns a boolean indicating whether or not we've completed the task.
	 * Abstract method must be overridden in derived classes.
	 *
	 * @return boolean indicating whether or not the task is complete
	 */
	protected abstract boolean IsComplete();

	/**
	 * Method called upon task deletion in order to handle cleaning up any
	 * potential unfinished items.  Abstract method must be overridden in
	 * derived classes.
	 */
    protected abstract void OnRemoveTask();

	/**
	 * Calls OnRemoveTask() in and only if this task has been started, otherwise
	 * does nothing.
	 */
    protected final void CleanUpTask()
    {
        // If the task has been started, perform cleanup activities
        if (taskStarted)
            OnRemoveTask();
    }

	/**
	 * Returns the name of this task.
	 *
	 * @return String indicating the name of this task
	 */
	public final String GetName()
	{
		return taskName;
	}

	/**
	 * Returns the name of the current state of the task.  Abstract method must
	 * be overridden by derrived classes.
	 *
	 * @return String indicating the current state of the task
	 */
	public abstract String GetStateName();

	/**
	 * Returns a boolean indicating whether or not it is OK to respond to the
	 * operator's joystick inputs (for driving).  Abstract method must be
	 * overridden by derrived classes.
	 *
	 * @return boolean indicating whether or not it is OK to respond to operator
	 * inputs
	 */
	protected abstract boolean OkToDrive();
}