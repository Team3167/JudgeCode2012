/*******************************************************************************
* File:  TaskManager.java
* Date:  1/17/2011
* Auth:  K. Loux
* Desc:  Class for managing robot tasks.  Contains the queue and also the
*        methods for performing tasks, evaluating whether or not to move to the
*        next task, etc.
*******************************************************************************/

// Declare our package
package judge.autonomous.tasks;

/**
 * Manages tasks by controlling the task queue.  Removes tasks as they are
 * completed, calls appropriate methods for performing and cleaning up tasks.
 *
 * @author K. Loux
 */
public class TaskManager
{
    // Fields
    // List of tasks to be completed
    private TaskQueue taskQueue;

    // Current task on which we're working
    private TaskBase currentTask;

    // Methods
    /**
	 * Constructor.  Creates tasks queue and initializes current task reference.
	 */
    public TaskManager()
    {
        // Create the tasks queue
        taskQueue = new TaskQueue();

        // Make the current task null
        currentTask = null;
    }

    /**
	 * Adds the specified task to the queue.
	 *
	 * @param newTask Task to be added to the queue
	 */
    public void AddTask(TaskBase newTask)
    {
        taskQueue.Push(newTask);
    }

	/**
	 * Performs the current task in the queue and checks to see if the task is
	 * complete.  If the task is complete, this removes the task from the queue
	 * and performs any necessary clean-up.
	 */
	public void DoCurrentTask()
	{
		try
		{
			// If we have something to do, try to get it done
			if (currentTask != null)
			{
				currentTask.PerformTaskActions();

				// Check to see if the current task is complete
				if (currentTask.IsComplete())
				{
					// Perform any cleanup actions associated with the task with
					// which we're finished
					currentTask.CleanUpTask();

					// If we have another task to do, make it the current task,
					// otherwise we make the current task null
					if (taskQueue.Size() > 0)
						currentTask = taskQueue.Pop();
					else
						currentTask = null;
				}
			}
			else if (taskQueue.Size() > 0)
				currentTask = taskQueue.Pop();
		}
		catch (Exception ex)
		{
			currentTask = null;
			System.err.println(ex.toString());
		}
	}

	/**
	 * Returns the name of the current task, if one exists.
	 *
	 * @return Name of the current task
	 */
	public String GetCurrentTaskName()
	{
		if (currentTask == null)
			// Names should be 21 characters long
            return "None                 ";
        
		return currentTask.GetName();
	}

	/**
	 * Returns the current task's state.
	 *
	 * @return State of the current task
	 */
	public String GetCurrentTaskState()
	{
		if (currentTask == null)
			return "                     ";

		return currentTask.GetStateName();
	}

	/**
	 * Removes all tasks from the queue, performing clean-up actions as
	 * necessary.
	 */
    public void ClearAllTasks()
    {
        TaskBase popedTask;
        while (taskQueue.Size() > 0)
        {
            // Remove the task from the queue and perform necessary clean-up
            // actions
			try
			{
				popedTask = taskQueue.Pop();
				popedTask.CleanUpTask();
			}
			catch (Exception ex)
			{
				System.err.println(ex.toString());
			}
        }

        // In addition to clearing the queue, set the current task to null
        currentTask = null;
    }

    /**
	 * Returns true if a current task exists.
	 *
	 * @return True if a current task exists, false otherwise
	 */
    public boolean PerformingTask()
    {
        // If we are doing a task or are about to do a task,
        // return true
        if (taskQueue.Size() > 0 || currentTask != null)
            return true;

        return false;
    }

	/**
	 * Polls the current task to see if it is OK to respond to operator inputs.
	 *
	 * @return True if it is OK to respond to operator inputs, false otherwise
	 */
	public boolean OkToDrive()
	{
		if (currentTask == null)
			return true;

		return currentTask.OkToDrive();
	}
}