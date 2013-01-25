/*******************************************************************************
* File:  WheelList.java
* Date:  1/12/2011
* Auth:  K. Loux
* Desc:  Fast array for Wheel objects.  Would prefer an ArrayList or Vector, but
*        FRC restrictions on java version require an alternative.
*******************************************************************************/

// Declare our package
package judge.drive;

/**
 * Variable size list of wheel objects.
 *
 * @author K. Loux
 */
public class WheelList
{
    // Fields
    private Wheel array[];
    private int arraySize = 0;

    // Methods
    /**
	 * Adds a wheel to the list.
	 *
	 * @param newWheel	Object to be added to the list
	 */
    public void Add(Wheel newWheel)
    {
        // Create a temporary array to store the current list
        int i;
        if (arraySize > 0)
        {
            Wheel tempArray[] = new Wheel[arraySize];
            for (i = 0; i < arraySize; i++)
                tempArray[i] = array[i];

            // Resize the current list
            array = new Wheel[arraySize + 1];

            // Repopulate the list
            for (i = 0; i < arraySize; i++)
                array[i] = tempArray[i];
        }
        else
        {
            i = 0;
            array = new Wheel[1];
        }

        // Add the new item
        array[i] = newWheel;

        // Increment the array size
        arraySize++;
    }

    /**
	 * Returns the object at the specified list index.
	 *
	 * @param i	List index from which to retrieve the wheel
	 *
	 * @return Specified wheel object
	 */
    public Wheel Get(int i)
    {
        return array[i];
    }

    /**
	 * Returns the number of wheels in the list
	 *
	 * @return Number of wheels in the list
	 */
    public int Size()
    {
        return arraySize;
    }
}