/*******************************************************************************
* File:  KalmanFilter.java
* Date:  1/9/2012
* Auth:  K. Loux
* Desc:  Class for estimating the state of a system with a Kalman filter.
*******************************************************************************/

// Declare our package
package judge.sensors;

// Imports
import edu.wpi.first.wpilibj.PIDSource;
import judge.math.Matrix;

/**
 * Class implementing a generic Kalman filter.
 * 
 * @author kerry
 */
public class KalmanFilter
{
    // The array of sensors
    private PIDSource[] sensors;
    private int sensorCount = 0;
    
	// The measurement information
    private Matrix observedState;// "z" vector (measurement vector)
    private Matrix observationMatrix;// "H" matrix (sensor model)
    private Matrix measurementNoiseCovariance;// "R" matrix (std. dev. squared)
	
	// The estimated state vector and its covariance
	private Matrix state;// "x" vector
    private Matrix stateCovariance;// "P" matrix (state uncertainty)
    private Matrix processNoiseCovariance;// "Q" matrix (std. dev. squared)
    
    // R and Q matrices describe how much trust to place in system model vs.
    // measurements.  Q describes modeling uncertainties, R describes
    // measurement uncertainties.
	
	// NOTE:  Current implementation throws exceptions if these matrices are changed
	// after they are set - this could possibly be removed to provide time-dependent
	// models of system dynamics and uncertainty.
	
	// State-space matricies
    private Matrix stateTransitionMatrix;// "A" matrix (system model)
    private Matrix inputMatrix;// "B" matrix (models effect of inputs on system)
	
	private boolean initialized = false;
	
	/**
	 * Default constructor for Kalman filter class
	 *
	 */
	public KalmanFilter()
	{
	}
	
	/**
	 * Constructor for Kalman filter class
	 * 
	 * @param initialState
	 * @param initialCovariance 
	 */
	public KalmanFilter(Matrix initialState, Matrix initialCovariance)
	{
		state = initialState;
        stateCovariance = initialCovariance;
	}
	
	/**
	 * Adds a sensor to the list that we use to update the state estimate.
	 * This is a simplistic implementation of a Kalman filter - assumptions
     * below:
	 * - we assume that every sensor is sampled at the same rate (also the
	 *   same rate that our model is updated).
     * - we assume that measurement noise covariance is constant
     * - we assume that process noise covariance is constant
	 * 
	 * @param sensor
     * @throws IllegalStateException 
	 */
	public void AddSensor(PIDSource sensor) throws IllegalStateException
	{
        // Make sure we're trying to add sensors after initializing the filter
		if (initialized)
			throw new IllegalStateException(
                    "Sensors must be added prior to Kalman filter initialization!");
        
        // Add the sensor to the array
        if (sensors == null || sensorCount == 0)
        {
            sensors = new PIDSource[1];
        }
        else if (sensors != null)// double check
        {
            PIDSource[] tempArray = new PIDSource[sensorCount + 1];
            System.arraycopy(sensors, 0, tempArray, 0, sensorCount);
            sensors = tempArray;
        }
        
        // Add the new sensor to the list
        sensors[sensorCount] = sensor;
        
        // Increment the number of sensors we have
        sensorCount++;
	}
	
	/**
	 * Initializes the filter - creates all of the required matrices and
	 * verifies the inputs.  Must be called prior to updating the state estimate.
     * @throws IllegalStateException 
	 */
	public void Initialize() throws IllegalStateException
	{
        // Make sure we're not callilng Initialize() twice
		if (initialized)
			throw new IllegalStateException("Kalman filter already initialized!");
		
		// Must have at least one sensor
		if (sensorCount == 0)
			throw new IllegalStateException(
					"Filter must be connected to at least one sensor!");
		
		// State must be defined
		if (state == null)
			throw new IllegalStateException(
					"State vector must be defined prior to initialization!");
		
		if (stateCovariance == null)
			if (state == null)
			throw new IllegalStateException(
					"State covariance matrix must be defined prior to initialization!");
		else if(stateCovariance.GetRowCount() != state.GetRowCount() ||
				stateCovariance.GetColCount() != state.GetRowCount())
			throw new IllegalStateException("State covariance matrix size "
					+ "mismatch:  Expected " + state.GetRowCount() + " x "
					+ state.GetRowCount() + ", found "
					+ stateCovariance.GetRowCount() + " x "
					+ stateCovariance.GetColCount() + ".");
		
		// Check dimensions (and existence) of all member matrices
		// Where possible, we will generate default matrices, but we print a
		// warning in case this wasn't the user's intention.
		if (measurementNoiseCovariance == null)
			throw new IllegalStateException(
					"Cannot create default measurement noise covariance matrix!");
		
		if (observationMatrix == null)
		{
			System.out.println("Warning:  Generating default observation matrix!");
			observationMatrix = Matrix.GetIdentity(state.GetRowCount());
		}
		else if (observationMatrix.GetRowCount() != sensorCount ||
				observationMatrix.GetColCount() != state.GetRowCount())
			throw new IllegalStateException("Observation matrix size mismatch:  "
					+ "Expected " + sensorCount + " x " + state.GetRowCount()
					+ ", found " + observationMatrix.GetRowCount() + " x "
					+ observationMatrix.GetColCount() + ".");
		
		if (processNoiseCovariance == null)
		{
			System.out.println(
					"Warning:  Generating default process noise covariance matrix!");
			processNoiseCovariance = Matrix.GetZero(state.GetRowCount());
		}
		else if (processNoiseCovariance.GetRowCount() != state.GetRowCount() ||
				processNoiseCovariance.GetColCount() != state.GetRowCount())
			throw new IllegalStateException("Process noise covariance matrix "
					+ "size mismatch:  Expected " + state.GetRowCount() + " x "
					+ state.GetRowCount() + ", found "
					+ processNoiseCovariance.GetRowCount() + " x "
					+ processNoiseCovariance.GetColCount() + ".");
		
		if (stateTransitionMatrix == null)
		{
			System.out.println(
					"Warning:  Generating default state transition matrix!");
			stateTransitionMatrix = Matrix.GetIdentity(state.GetRowCount());
		}
		else if (stateTransitionMatrix.GetRowCount() != state.GetRowCount() ||
				stateTransitionMatrix.GetColCount() != state.GetRowCount())
			throw new IllegalStateException("State transition matrix "
					+ "size mismatch:  Expected " + state.GetRowCount() + " x "
					+ state.GetRowCount() + ", found "
					+ stateTransitionMatrix.GetRowCount() + " x "
					+ stateTransitionMatrix.GetColCount() + ".");
		
		if (inputMatrix == null)
		{
			System.out.println("Warning:  Generating default input matrix!");
			inputMatrix = Matrix.GetZero(state.GetRowCount(), 1);
			// NOTE:  Not sure if this is wise - we don't know how big the input
			// vector will be
		}
		// No size check on input matrix - we don't yet know how big the input
		// vector will be
		
		// Create the observation matrix (internal use only, but we didn't
		// know the size until now)
		observedState = new Matrix(sensorCount, 1);
		
		// Set the initialization flag
		initialized = true;
	}
	
	/**
	 * Assigns the initial state vector (x)
	 * 
	 * @param x
	 * @throws IllegalStateException if filter was previously initialized
	 */
	public void SetStateVector(Matrix x) throws IllegalStateException
	{
		if (initialized)
			throw new IllegalStateException(
					"x vector must be set prior to Kalman filter initialization!");
		
		state = x;
	}
	
	/**
	 * Assigns the initial state covariance matrix (P)
	 * Represents uncertainty in state estimate
	 * 
	 * @param p
	 * @throws IllegalStateException if filter was previously initialized
	 */
	public void SetStateCovarianceMatrix(Matrix p) throws IllegalStateException
	{
		if (initialized)
			throw new IllegalStateException(
					"P matrix must be set prior to Kalman filter initialization!");
		
		stateCovariance = p;
	}
	
	/**
	 * Assigns the measurement noise covariance matrix (R)
	 * Represents uncertainty in sensor model
	 * 
	 * @param r
	 * @throws IllegalStateException if filter was previously initialized
	 */
	public void SetMeasurementNoiseCovariance(Matrix r) throws IllegalStateException
	{
		if (initialized)
			throw new IllegalStateException(
					"R matrix must be set prior to Kalman filter initialization!");
		
		measurementNoiseCovariance = r;
	}
	
	/**
	 * Assigns the observation matrix (H)
	 * This is the sensor model (function of state)
	 * 
	 * @param h
	 * @throws IllegalStateException if filter was previously initialized
	 */
	public void SetObservationMatrix(Matrix h) throws IllegalStateException
	{
		if (initialized)
			throw new IllegalStateException(
					"H matrix must be set prior to Kalman filter initialization!");

		observationMatrix = h;
	}
	
	/**
	 * Assigns the process noise covariance matrix (Q)
	 * Represents uncertainty of the system model
	 * 
	 * @param q
	 * @throws IllegalStateException if filter was previously initialized
	 */
	public void SetProcessNoiseCovariance(Matrix q) throws IllegalStateException
	{
		if (initialized)
			throw new IllegalStateException(
					"Q matrix must be set prior to Kalman filter initialization!");

		processNoiseCovariance = q;
	}
	
	/**
	 * Assigns the state transition matrix (A)
	 * This is the system model (function of state and inputs)
	 * 
	 * @param a
	 * @throws IllegalStateException if filter was previously initialized
	 */
	public void SetStateTransitionMatrix(Matrix a) throws IllegalStateException
	{
		if (initialized)
			throw new IllegalStateException(
					"A matrix must be set prior to Kalman filter initialization!");

		stateTransitionMatrix = a;
	}
	
	/**
	 * Assigns the input matrix (B)
	 * Maps the inputs to states (the effect of the input on the system)
	 * 
	 * @param b
	 * @throws IllegalStateException if filter was previously initialized
	 */
	public void SetInputMatrix(Matrix b) throws IllegalStateException
	{
		if (initialized)
			throw new IllegalStateException(
					"B matrix must be set prior to Kalman filter initialization!");

		inputMatrix = b;
	}
	
	/**
	 * Gets the current sensor measurements and updates the state estimate.
	 * 
	 * @param input "u" vector
	 * @return the estimated state matrix
     * @throws IllegalStateException 
	 */
	public Matrix UpdateEstimate(Matrix input) throws IllegalStateException
	{
        // Make sure we've called initialize
		if (!initialized)
			throw new IllegalStateException("Kalman filter not initialized!");
        
        // Predict state vector and covariance
        state = stateTransitionMatrix.Multiply(state).Add(
                inputMatrix.Multiply(input));
        stateCovariance = stateTransitionMatrix.Multiply(stateCovariance)
                .Multiply(stateTransitionMatrix.GetTranspose()).Add(
                processNoiseCovariance);
        
        // Compute Kalman gain
        Matrix k = stateCovariance.Multiply(observationMatrix.GetTranspose())
                .Multiply(observationMatrix.Multiply(stateCovariance).Multiply(
                observationMatrix.GetTranspose()).Add(measurementNoiseCovariance));
        
        // Update the measurement vector
        int i;
        for (i = 0; i < sensorCount; i++)
        {
            observedState.SetElement(i, 0, sensors[i].pidGet());
        }
        
        // Correct the prediction based on measurements
        state = state.Add(k.Multiply(observedState.Subtract(
                observationMatrix.Multiply(state))));
        stateCovariance = stateCovariance.Subtract(k.Multiply(
                observationMatrix.Multiply(stateCovariance)));
        
        return state;
	}
	
	/**
	 * Gets the number of states
	 * 
	 * @return rows of state vector
	 */
	public int GetStateSize()
	{
		return state.GetRowCount();
	}
	
	/**
	 * Gets the number of inputs expected
	 * 
	 * @return columns of input matrix
	 */
	public int GetInputSize()
	{
		return inputMatrix.GetColCount();
	}
}
