/*******************************************************************************
* File:  TaskBridge.java
* Date:  1/11/2012
* Auth:  K. Loux
* Desc:  Task for raising or lowering the teetering bridges.
*******************************************************************************/

// Declare our package
package judge.autonomous.tasks;

/**
 * Task for raising or lowering the teetering bridges.
 * 
 * @author kloux
 */
public class TaskBridge extends TaskBase
{
	// Action enum
	public static final int kBridgeLower = 0;
	public static final int kBridgeRaise = 1;
	public final int bridgeAction;
	
	// States
	private static final byte stateFindBridge = 0;
	private static final byte stateAlignToBridge = 1;
	private static final byte stateMoveToActionPosition = 2;
	private static final byte stateActOnBridge = 3;
	
	/**
	 * Constructor for bridge tasks
	 * 
	 * @param action 
	 */
	public TaskBridge(int action)
	{
		super("Bridge Work          ");
		bridgeAction = action;
		
		SetNextState(stateFindBridge);
	}
	
	/**
	 * Override of main state machine method for doing work
	 */
	protected void ProcessState()
	{
		switch (GetState())
		{
			case stateFindBridge:
				break;
				
			case stateAlignToBridge:
				break;
				
			case stateMoveToActionPosition:
				break;
				
			case stateActOnBridge:
				break;
		}
	}
	
	/**
	 * Determines if we've finished with the bridge
	 * @return 
	 */
	protected boolean IsComplete()
	{
		// FIXME:  Implement
		return false;
	}

	/**
	 * Nothing required for this task
	 */
    protected void OnRemoveTask()
	{
	}
	
	/**
	 * Gets the name of the current state, processed for LCD output
	 * 
	 * @return state name
	 */
	public String GetStateName()
	{
		switch (GetState())
		{
			case stateFindBridge:
				return "Finding Bridge       ";
				
			case stateAlignToBridge:
				return "Aligning To Bridge   ";
				
			case stateMoveToActionPosition:
				return "Moving Into Position ";
				
			case stateActOnBridge:
				if (bridgeAction == kBridgeLower)
					return "Lowering Bridge      ";
				else
					return "Raising Bridge       ";
				
			default:
				return "Unknown state        ";
		}
	}

	/**
	 * Checks to see if we should allow the driver to control the robot.
	 * 
	 * @return 
	 */
	protected boolean OkToDrive()
	{
		if (GetState() == stateFindBridge)
			return true;

		return false;
	}
}
