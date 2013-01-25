/*******************************************************************************
* File:  TaskQueue.java
* Date:  1/13/2011
* Auth:  K. Loux
* Desc:  Queue for storing robot tasks.
*******************************************************************************/

// Declare our package
package judge.autonomous.tasks;

// Standard java imports
import java.util.EmptyStackException;

/**
 * Queue for task objects.
 *
 * @author K. Loux
 */
public class TaskQueue
{
    // Fields
	// Current number of tasks
	private int taskCount = 0;

	// Array of tasks
	private TaskBase pendingTasks[];

	// Methods
	/**
	 * Adds a task to the queue.
	 *
	 * @param newTask Task to be added to the queue
	 */
	public void Push(TaskBase newTask)
	{
		// Create a temporary array to store the current list
        int i;
        if (taskCount > 0)
        {
            TaskBase tempArray[] = new TaskBase[taskCount];
            for (i = 0; i < taskCount; i++)
                tempArray[i] = pendingTasks[i];

            // Resize the current list
            pendingTasks = new TaskBase[taskCount + 1];

            // Repopulate the list
            for (i = 0; i < taskCount; i++)
                pendingTasks[i] = tempArray[i];
        }
        else
        {
            i = 0;
            pendingTasks = new TaskBase[1];
        }

        // Add the new item
        pendingTasks[i] = newTask;

        // Increment the array size
        taskCount++;
	}

	/**
	 * Returns the next task and removes it from the queue.
	 *
	 * @return Next task in the queue
	 *
	 * @throws EmptyStackException
	 */
	public TaskBase Pop() throws EmptyStackException
	{
        // If we don't have any tasks, this isn't really a valid call
        if (taskCount <= 0)
			throw new EmptyStackException();

        // Store the next task temporarily
        TaskBase nextTask = pendingTasks[0];

		// Create a temporary array to store the current list
        TaskBase tempArray[] = new TaskBase[taskCount - 1];
        int i;
        for (i = 1; i < taskCount; i++)
            tempArray[i - 1] = pendingTasks[i];

        // Decrement the array size
        taskCount--;
        
        // Resize the current list
        pendingTasks = new TaskBase[taskCount];
        
        // Repopulate the list
        for (i = 0; i < taskCount; i++)
            pendingTasks[i] = tempArray[i];

        return nextTask;
	}

	/**
	 * Returns the number of tasks in the queue.
	 *
	 * @return Number of pending tasks
	 */
    public int Size()
    {
        return taskCount;
    }
}