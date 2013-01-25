/*******************************************************************************
* File:  TaskBalance.java
* Date:  1/11/2012
* Auth:  K. Loux
* Desc:  Task for balancing the teetering bridges.
*******************************************************************************/

// Declare our package
package judge.autonomous.tasks;

// Local imports
import judge.math.Matrix;
import judge.BalanceFilter;
import judge.RobotConfiguration;
import judge.util.PIDControllerII;

/**
 *
 * @author kloux
 */
public class TaskBalance extends TaskBase
{
	// Fields
	private Matrix state, lastInput;
	private BalanceFilter estimator;
	private PIDControllerII controller;
	
	/**
	 * Constructor for balancing task
	 */
	public TaskBalance(BalanceFilter stateEstimator)
	{
		super("Bridge Balancing     ");
		
		estimator = stateEstimator;
		state = new Matrix(estimator.GetStateSize(), 1);
		lastInput = new Matrix(estimator.GetInputSize(), 1);
		lastInput.MakeZero();
		controller = new PIDControllerII(RobotConfiguration.balanceKp,
				RobotConfiguration.balanceKi,
				RobotConfiguration.balanceQueueSize,
				RobotConfiguration.frequency);
	}
	
	/**
	 * Balancing task has only one state
	 */
	protected void ProcessState()
	{
		// State is [bridge angle;
		//           brige rotation rate;
		//           robot position;
		//           robot velocity;
		//           robot acceleration]
		//
		// Inputs are [fore-aft speed command;
		//             last fore-aft speed command]
		
		// Update the state estimate
		state = estimator.UpdateEstimate(lastInput);
		
		// Shift speed command to last speed command and get new speed command
		lastInput.SetElement(1, 0, lastInput.GetElement(0, 0));
		lastInput.SetElement(0, 0, controller.DoControl(state.GetElement(0, 0)));
		
		// Act on the new command
		// FIXME:  Implement this
	}
	
	/**
	 * No reason to stop balancing until the match is over
	 * 
	 * @return false
	 */
	protected boolean IsComplete()
	{
		return false;
	}

	/**
	 * Nothing required
	 */
    protected void OnRemoveTask()
	{
	}
	
	/**
	 * Only one state for balancing task.
	 * 
	 * @return 
	 */
	public String GetStateName()
	{
		return "Moving...            ";
	}

	/**
	 * Always returns false - never let the user drive in this mode
	 * 
	 * @return false
	 */
	protected boolean OkToDrive()
	{
		return false;
	}
}
