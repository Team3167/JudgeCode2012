/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// This package
package judge;

// WPI imports
import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Kinect;

import edu.wpi.first.wpilibj.RobotDrive;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.DriverStation;

/*import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;*/

import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogModule;

//import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.camera.AxisCamera;

// Standard Java imports
import java.util.Timer;
import java.util.TimerTask;

// Judge imports
import judge.drive.*;
import judge.autonomous.tasks.*;
import judge.sensors.*;
import judge.util.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation.
 *
 * The package for this class was changed by modifying the manifest file in the
 * resource directory.
 *
 * @author K. Loux
 */
public class Team3167Robot extends IterativeRobot
{
    // Fields
	// The number of autonomous modes we have
    private static final int AUTO_MODE_COUNT = 3;

    // Currently selected autonomous profile
    private int autonomousProfile = 0;

    // Robot task manager
    private TaskManager taskManager;

    // HID objects
    private Joystick driver;
	private Joystick gunner;
	private Kinect kinect;

	//private CameraWrapper camera = new CameraWrapper();
	private AxisCamera camera = AxisCamera.getInstance();

    // Robot drive object
    private HolonomicRobotDrive drive;
	//private HolonomicPositioner positioner;
	//private OmniDrive drive;

	/*private RobotDrive drive = new RobotDrive(
			RobotConfiguration.leftFrontMotorChannel,
			RobotConfiguration.leftRearMotorChannel,
			RobotConfiguration.rightFrontMotorChannel,
			RobotConfiguration.rightRearMotorChannel);
	private Gyro gyro = new Gyro(RobotConfiguration.yawGyroChannel);*/

	// For displaying information on the driver's station message window
	private final DriverStationLCD msg = DriverStationLCD.getInstance();
	private Timer timer;
    private float matchTime;
	private DSPlotDataBuffer dsBuffer;

    // Objects for debouncing switches
    private JoystickButton trigger, button2Driver, button2Gunner, button3,button3Gunner,
			button4, button4Gunner, button5, button5Gunner, button6, button6Gunner,
			button7, button7Gunner, button8, button9, button9Gunner, button10, button11, button11Gunner, button12;

	private double speed = 0.0;
	// Pneumatic objects
    //private Compressor compressor;

	// Specialized objects
	/*private Turret turret;
	private BalanceFilter balanceState;*/
	private Pitcher pitcher;
	private Bucket bucket;
	private boolean bucketSwitch = false;

	// Routine to call all periodic methods for autonomous and teleoperated
	private void ExecuteCommonRoutines()
	{
		taskManager.DoCurrentTask();
		//turret.Update();
		pitcher.Update();
		bucket.Update();

		//UpdateDSDataCluster();
	}

    // Robot initialization method =============================================
	/**
	 * Robot initialization method, called once when the cRIO is first powered
	 * up.
	 */
    public void robotInit()
    {
		// Create the joystick
		driver = new Joystick(1);
		gunner = new Joystick(2);

		// Create the button objects
        button3 = new JoystickButton(driver, 3);
        button4 = new JoystickButton(driver, 4);
        button5 = new JoystickButton(driver, 5);
        button6 = new JoystickButton(driver, 6);
        button7 = new JoystickButton(driver, 7);
        button8 = new JoystickButton(driver, gunner, 8);
		button9 = new JoystickButton(driver, 9);
		button10 = new JoystickButton(driver, gunner, 10);
		button11 = new JoystickButton(driver, 11);
        button12 = new JoystickButton(driver, gunner, 12);

		button2Driver = new JoystickButton(driver, 2);

		trigger = new JoystickButton(gunner, 1);
		button2Gunner = new JoystickButton(gunner, 2);
		button3Gunner = new JoystickButton(gunner, 3);
	    button4Gunner = new JoystickButton(gunner, 4);
		button5Gunner = new JoystickButton(gunner, 5);
		button6Gunner = new JoystickButton(gunner, 6);
		button7Gunner = new JoystickButton(gunner, 7);
		button9Gunner = new JoystickButton(gunner, 9);
		button11Gunner = new JoystickButton(gunner, 11);


        // Create the buffer for sending data to the dashboard
		dsBuffer = new DSPlotDataBuffer();

		// Create sensors
		/*ADXL345_I2C accelerometer = new ADXL345_I2C(
				RobotConfiguration.digitalSideCarSlot,
				ADXL345_I2C.DataFormat_Range.k2G);
		AccelWrapper accelPID = new AccelWrapper(accelerometer,
				ADXL345_I2C.Axes.kX);

		RateGyro pitchGyro = new RateGyro(RobotConfiguration.analogInputSlot,
				RobotConfiguration.pitchGyroChannel);

		// FIXME:  Wheel encoders, or something to average multiple wheel encoders?

		// Initialize specialized objects
		turret = new Turret(driver, gunner);*/
		//balanceState = new BalanceFilter(accelPID, pitchGyro, someEncoder);// FIXME:  Implement

		pitcher = new Pitcher();
		bucket = new Bucket();

        // Initialize drive object and create the positioner
          InitializeHolonomicDrive();
		//positioner = new HolonomicPositioner(drive, freq);
		//InitializeOmniDrive();

        // Create the compressor object
        /*compressor = new Compressor(RobotConfiguration.digitalSideCarSlot,
                RobotConfiguration.pressureSwitchChannel,
                RobotConfiguration.pwmOutputSlot,
                RobotConfiguration.relayChannel);
        compressor.start();*/

        // Create the task manager object
        taskManager = new TaskManager();

		// Create a thread for displaying the LCD data to keep it from slowing
		// down the critical operation of the robot
		timer = new Timer();
		timer.schedule(new TimerTask()
		{
			public void run()
			{
				UpdateLCD();
			}
		}, 0, 500);// 0 - no delay (start right now), 500 - period (in msec)

		// Print to the console to tell us that we're ready to rock and roll
		// Useful when debugging and we don't want to wait longer than necessary
		// to make sure the robot is ready
		System.out.println("Robot is ready to rock and roll!");
    }

    // Autonomous mode methods =================================================
	/**
	 * Initialization method for autonomous mode.  Called once when mode is
	 * first set to autonomous.
	 */
    public void autonomousInit()
    {
        // Clear out any tasks from a previous practice/test
        taskManager.ClearAllTasks();

        // Reset the match time
        matchTime = 0.0f;

        // Reset the controllers so we don't run away
        //drive.Reset();

		pitcher.Set(0.3);

		// Depending on the selected autonomous mode, add a bunch of tasks to
		// the queue
		// Tuning and troubleshooting profiles
        if (autonomousProfile == 0)
        {
			taskManager.AddTask(new TaskAutonomous(pitcher, bucket,
					TaskAutonomous.Distance.distanceShort));
        }
        else if (autonomousProfile == 1)
        {
			taskManager.AddTask(new TaskAutonomous(pitcher, bucket,
					TaskAutonomous.Distance.distanceMedium));
        }
		else if (autonomousProfile == 2)
        {
			taskManager.AddTask(new TaskAutonomous(pitcher, bucket,
					TaskAutonomous.Distance.distanceLong));
        }
    }

	/**
	 * Method called as fast as possible while in autonomous mode.  Empty
	 * method.
	 */
    public void autonomousContinuous()
    {
		// Nothing here
    }

	/**
	 * Method called at 50 Hz while in autonomous mode.  Performs tasks, updates
	 * the elevator and prints messages to the dashboard.
	 */
    public void autonomousPeriodic()
    {
		ExecuteCommonRoutines();
    }

    // Teleoperated mode methods ===============================================
	/**
	 * Initialization method for teleoperated mode.  Called once when mode is
	 * first set to teleoperated.
	 */
    public void teleopInit()
    {
		taskManager.ClearAllTasks();

		//bucket.SetExtendBridgeTipper(true);

        // Reset the match time
        matchTime = 0.0f;

		//bucketSwitch = true;

		speed = 0.0;
    }

	/**
	 * Method called as fast as possible while in teleoperated mode.  Handles
	 * button presses only.
	 */
    public void teleopContinuous()
    {
		if(bucketSwitch)
		{
		bucket.SetExtendBridgeTipper(true);
		bucketSwitch = false;
		}
		// TODO:  Implement button code
		// Gunner buttons
        if (trigger.HasJustBeenPressed())
			pitcher.Shoot();

		/*if (button2Gunner.HasJustBeenPressed())
			turret.ToggleMode();*/

		// Driver buttons
		if (button2Driver.HasJustBeenPressed())
			taskManager.ClearAllTasks();

		// Buttons for both sticks
		if (button6.HasJustBeenPressed())
		{
			//taskManager.AddTask(new TaskBridge(TaskBridge.kBridgeLower));
			bucket.SetExtendBridgeTipper(true);
		}

		if (button4.HasJustBeenPressed())
		{
			//taskManager.AddTask(new TaskBalance(balanceState));
			bucket.SetExtendBridgeTipper(false);
		}

		if (button5.IsPressed())
			bucket.Set(Bucket.Move.up);
		else if (button3.IsPressed())
			bucket.Set(Bucket.Move.down);
		else
			bucket.Set(Bucket.Move.stop);

		if(button3Gunner.HasJustBeenPressed())
		{
			if(speed == 0.0)
				speed = 10.0;
			else
				speed = 0.0;
		}

		if(button4Gunner.HasJustBeenPressed())
		{
			if(speed == 0.0)
				speed = 20.0;
			else
				speed = 0.0;
		}
		if(button5Gunner.HasJustBeenPressed())
		{
			if(speed == 0.0)
				speed = 30.0;
			else
				speed = 0.0;
		}

		if(button6Gunner.HasJustBeenPressed())
		{
			if(speed == 0.0)
				speed = 40.0;
		 	else
				speed = 0.0;
		}

		if(button7Gunner.HasJustBeenPressed())
		{
			pitcher.setBackSpin(1.20);
		}

		else if(button9Gunner.HasJustBeenPressed())
		{
			pitcher.setBackSpin(1.3);
		}

		else if(button11Gunner.HasJustBeenPressed())
		{
			pitcher.setBackSpin(1.5);
		}
    }

	private final double pitcherStartCommand = 10.0;// [ft]
	private final double pitcherEndCommand = 40.0;// [ft]
	private final double throttleDeadband = 0.05;// [-]
	private final double throttleCmdSlope =
			(pitcherEndCommand - pitcherStartCommand) /
			(1.0 - throttleDeadband);// [ft/%]
	private final double throttleCmdIntercept = pitcherStartCommand
			- throttleCmdSlope * throttleDeadband;// [ft]

	/**
	 * Method called at 50 Hz while in teleoperated mode.  Drives robot
	 * according to joystick inputs, performs tasks, and prints messages to
     * the dashboard.
	 */
    public void teleopPeriodic()
    {
		// Move the 'bot according to the joystick position, but only if we have
        // no tasks to do
        if (taskManager.OkToDrive())
            drive.Drive(driver);
		/*drive.mecanumDrive_Cartesian(driver.getX(), driver.getY(),
				driver.getTwist(), gyro.getAngle());*/

		if (speed > 0.0)
			pitcher.Set(speed - (5.0 * gunner.getThrottle()));
		else
			pitcher.Set(0.0);

		// Update manual pitcher command
		/*double value = (-gunner.getThrottle() + 1.0) * 0.5;
		if (value < throttleDeadband)
			pitcher.Set(0.0);
		else
			pitcher.Set(value * throttleCmdSlope + throttleCmdIntercept);*/

		ExecuteCommonRoutines();
    }

    // Disabled mode methods ===================================================
	/**
	 * Initialization method for disabled mode.  Called once every time the mode
	 * changes from running (either autonomous or teleoperated) to disabled.
	 */
    public void disabledInit()
    {
		// We should probably clear any left-over tasks, so we don't have
        // competing logic inputs
        taskManager.ClearAllTasks();
    }

	/**
	 * Method called as fast as possible while the robot is disabled.  Handles
	 * button presses for switching autonomous profiles.
	 */
    public void disabledContinuous()
    {
        // If the trigger is pressed, increment the selected autonomous profile
        if (trigger.HasJustBeenPressed())
        {
            autonomousProfile++;

            // Make sure the profile number is valid by resetting it to zero
            // when necessary
            if (autonomousProfile >= AUTO_MODE_COUNT)
                autonomousProfile = 0;
        }
    }

	/**
	 * Method called at 50 Hz while the robot is disabled.
	 */
    public void disabledPeriodic()
    {
        //UpdateDSDataCluster();
    }

    // Private methods =========================================================
	/**
	 * Initialization method for the
	 *
	 * {@link HolonomicRobotDrive} object.  Sets up the gear ratios, controller
	 * gains, encoder parameters and wheel geometry for each of the wheels.
	 * Initialization fails if the
	 *
	 * {@link HolonomicRobotDrive} is unable to find a solution in which all of
	 * the robot's degrees of freedom can be controlled.
	 *
	 * @throws IllegalStateException
	 */
    private void InitializeHolonomicDrive() throws IllegalStateException
    {
        // Define local variables (to make this easier to read)
        double frontKp = 0.1 * 0.8;
        double frontKi = 0.0;
        double rearKp = 0.15 * 0.8;
        double rearKi = 0.0;
        double integralTime = 0.0;

        double wheelRadius = 4.0;// [in]
        double rollerAngle = 45;// [deg]

        double rightAxisX = 1.0;// [-]
        double leftAxisX = -1.0;// [-]
        double axisY = 0.0;// [-]

		double halfTrack = 11.5;// [in]
		double cgPercent = 0.5;// [%] (positive rearwards)
		double wheelbase = 28.0;// [in]
		double frontToCG = wheelbase * cgPercent;// [in]
        double rearToCG = frontToCG - wheelbase;// [in]

		///double rateFilterOmega = 3.0;// [Hz]
		//double rateFilterZeta = 1.0;// [-]

        //boolean reverseEncoder = true;

		// Calculate the velocity limit
        double maxMotorSpeed = RobotConfiguration.cimMotorMaxSpeed;// [RPM]
        double velocityScale = 0.85;// we only want to use xx% of motor speed
		double maxWheelSpeed = maxMotorSpeed * velocityScale
				/ RobotConfiguration.drivetrainP80GearboxRatio
                * 2.0 * Math.PI / 60.0;// [rad/sec]

        // Create the drive object
        drive = new HolonomicRobotDrive(RobotConfiguration.frequency,
				RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.analogInputModule);

        int motorChannel, encChanA, encChanB;

        // Add wheels to the drive
        // Left front wheel
        drive.AddWheel(-halfTrack, frontToCG, leftAxisX, axisY,
                    rollerAngle, wheelRadius,
                    RobotConfiguration.digitalSideCarModule,
					RobotConfiguration.leftFrontMotorChannel, maxWheelSpeed);

        // Right front wheel
        drive.AddWheel(halfTrack, frontToCG, rightAxisX, axisY,
                    -rollerAngle, wheelRadius,
                    RobotConfiguration.digitalSideCarModule,
					RobotConfiguration.rightFrontMotorChannel, maxWheelSpeed);

        // Left rear wheel
        drive.AddWheel(-halfTrack, rearToCG, leftAxisX, axisY,
                    rollerAngle, wheelRadius,
                    RobotConfiguration.digitalSideCarModule,
					RobotConfiguration.leftRearMotorChannel, maxWheelSpeed);

        // Right rear wheel
       drive.AddWheel(halfTrack, rearToCG, rightAxisX, axisY,
                    -rollerAngle, wheelRadius,
                    RobotConfiguration.digitalSideCarModule,
					RobotConfiguration.rightRearMotorChannel, maxWheelSpeed);

		// Add acceleration limits
		drive.SetFrictionCoefficient(0.7);// Creates acceleration limit

		// Set the deadband for the joysticks
		drive.SetDeadband(0.1);// horizontal deadband
		drive.SetMinimumOutput(0.1);// vertical deadband

		// Pass the plot buffer so we can plot wheel response
		drive.SetDSBuffer(dsBuffer);

        // Initialize the drive object
        if (!drive.Initialize())
			throw new IllegalStateException(
					"HolonomicRobotDrive initialization failed!");
    }

	/**
	 * Initialization method for the
	 *
	 * {@link OmniDrive} object.
	 */
	/*private void InitializeOmniDrive()
	{
		drive = new OmniDrive(RobotConfiguration.frequency,
				RobotConfiguration.digitalSideCarModule,
				RobotConfiguration.analogInputModule);

		double wheelRadius = 4.0;// [in]
        double rollerAngle = 90;// [deg]

		double rightAxisX = 1.0;// [-]
        double leftAxisX = -1.0;// [-]
		double halfTrack = 7.5;// [in]

		// Calculate the velocity limit
        double maxMotorSpeed = RobotConfiguration.cimMotorMaxSpeed;// [RPM]
        double velocityScale = 0.85;// we only want to use xx% of motor speed
		double maxWheelSpeed = maxMotorSpeed * velocityScale
				/ RobotConfiguration.drivetrainP80GearboxRatio
                * 2.0 * Math.PI / 60.0;// [rad/sec]

		// Add wheels to the drive
        // Left front wheel
        drive.AddWheel(-halfTrack, 0.0, leftAxisX, 0.0,
                    0.0, wheelRadius,
                    RobotConfiguration.digitalSideCarModule,
					RobotConfiguration.leftFrontMotorChannel, maxWheelSpeed);

        // Right front wheel
        drive.AddWheel(halfTrack, 0.0, rightAxisX, 0.0,
                    0.0, wheelRadius,
                    RobotConfiguration.digitalSideCarModule,
					RobotConfiguration.rightFrontMotorChannel, maxWheelSpeed);

        // Left rear wheel
        drive.AddWheel(-halfTrack, 0.0, leftAxisX, 0.0,
                    rollerAngle, wheelRadius,
                    RobotConfiguration.digitalSideCarModule,
					RobotConfiguration.leftRearMotorChannel, maxWheelSpeed);

        // Right rear wheel
       drive.AddWheel(halfTrack, 0.0, rightAxisX, 0.0,
                    rollerAngle, wheelRadius,
                    RobotConfiguration.digitalSideCarModule,
					RobotConfiguration.rightRearMotorChannel, maxWheelSpeed);

		// Add acceleration limits
		drive.SetFrictionCoefficient(0.7);// Creates acceleration limit

		// Set the deadband for the joysticks
		drive.SetDeadband(0.1);// horizontal deadband
		drive.SetMinimumOutput(0.1);// vertical deadband

		if (!drive.Initialize())
			System.out.println("Drive initialization failed!");
	}*/

	/**
	 * Returns the name of the profile with the specified index.  Names have 21
	 * characters (including spacers) as to provide an easy means of displaying
	 * on the Driver's Station's LCD area.
	 *
	 * @param profileIndex	The index specifying the desired profile
	 *
	 * @return String of 21 characters containing the name of the profile
	 */
    private String GetAutonomousProfileName(final int profileIndex)
    {
        if (profileIndex == 0)
            return "Short Distance Mode  ";
		else if (profileIndex == 1)
			return "Medium Distance Mode ";
        else if (profileIndex == 2)
            return "Long Distance Mode   ";

        return "Profile not named    ";
    }

	/**
	 * Updates the text being displayed on the Driver's Station's LCD area.
	 * Calls different methods for generating the text depending on the robot's
	 * current state of operation.
	 */
	private synchronized void UpdateLCD()
	{
        // It is possible to be autonomous and disabled, even though we treat
        // these as three states, so we add checks here to make it work as
        // expected
		if (isAutonomous() && !isDisabled())
			UpdateAutonomousLCD();
		else if (isOperatorControl() && !isDisabled())
			UpdateTeleoperatedLCD();
		else// Disabled
			UpdateDisabledLCD();

		// Send the changes to the driver's station
		msg.updateLCD();
	}

	/**
	 * Generates the LCD text to be displayed during autonomous mode.  All text
	 * is 21 characters per line in order to ensure erasure of previously
	 * displayed text.
	 */
	private void UpdateAutonomousLCD()
	{
		// Show the name of the currently selected autonomous profile
		msg.println(DriverStationLCD.Line.kMain6, 1, "     AUTONOMOUS      ");
		msg.println(DriverStationLCD.Line.kUser2, 1,
				GetAutonomousProfileName(autonomousProfile));
		msg.println(DriverStationLCD.Line.kUser3, 1, "                     ");//turret.GetStateName());
		msg.println(DriverStationLCD.Line.kUser4, 1, "TASK:                ");
		msg.println(DriverStationLCD.Line.kUser5, 1,
				taskManager.GetCurrentTaskName());
		msg.println(DriverStationLCD.Line.kUser6, 1,
				taskManager.GetCurrentTaskState());
	}

	/**
	 * Generates the LCD text to be displayed during teleoperated mode.  All
	 * text is 21 characters per line in order to ensure erasure of previously
	 * displayed text.
	 */
	private void UpdateTeleoperatedLCD()
	{
	    msg.println(DriverStationLCD.Line.kMain6, 1, "    TELEOPERATED     ");
		String s;
		Integer i = new Integer((int)(pitcher.GetSpeed() * 100.0));
		s = i.toString();
		if((int)(pitcher.GetSpeed() * 100.0) >= 40 && (int)(pitcher.GetSpeed() * 100.0) <= 45)
		{
    	msg.println(DriverStationLCD.Line.kUser2, 1, s + "  Middle                     ");
		}
		else if((int)(pitcher.GetSpeed() * 100.0) >= 50 && (int)(pitcher.GetSpeed() * 100.0) <= 55)
		{
    	msg.println(DriverStationLCD.Line.kUser2, 1, s + "  Top                     ");
		}
		else if((int)(pitcher.GetSpeed() * 100.0) >= 30 && (int)(pitcher.GetSpeed() * 100.0) <= 35)
		{
    	msg.println(DriverStationLCD.Line.kUser2, 1, s + "  Bottom                     ");
		}
		else
		{
    	msg.println(DriverStationLCD.Line.kUser2, 1, s + "                     ");
		}
		if (bucket.IsDown())
			msg.println(DriverStationLCD.Line.kUser3, 1, "Bucket DOWN!         ");//turret.GetStateName());
		else
			msg.println(DriverStationLCD.Line.kUser3, 1, "                     ");
		msg.println(DriverStationLCD.Line.kUser4, 1, "TASK:                ");
		msg.println(DriverStationLCD.Line.kUser5, 1,
				taskManager.GetCurrentTaskName());
		msg.println(DriverStationLCD.Line.kUser6, 1,
				taskManager.GetCurrentTaskState());
	}

	/**
	 * Generates the LCD text to be displayed during disabled mode.  All text
	 * is 21 characters per line in order to ensure erasure of previously
	 * displayed text.
	 */
	private void UpdateDisabledLCD()
	{
		// Show the name of the currently selected autonomous profile
		msg.println(DriverStationLCD.Line.kMain6, 1, "      DISABLED       ");
		msg.println(DriverStationLCD.Line.kUser2, 1, "                     ");
		msg.println(DriverStationLCD.Line.kUser3, 1, "AUTONOMOUS PROFILE:  ");
		msg.println(DriverStationLCD.Line.kUser4, 1,
				GetAutonomousProfileName(autonomousProfile));
		msg.println(DriverStationLCD.Line.kUser5, 1, "                     ");
		msg.println(DriverStationLCD.Line.kUser6, 1, "                     ");
	}

	/**
	 * Creates the low-priority data packet to be sent to the Driver's Station
	 * for display.  This is used in place of the default method in order to
	 * support the custom plotting capabilities added to the Driver's Station
	 * application (programmed in LabVIEW).
	 */
	/*private void UpdateDSDataCluster()
	{
        // Update the match time
        matchTime += RobotConfiguration.timeStep;

		// Create the cluster structure as required to send data to the
		// dashboard
		int i;
		Dashboard lowDashData = DriverStation.getInstance().getDashboardPackerLow();
        lowDashData.addCluster();
        {
			// Analog modules
            lowDashData.addCluster();
            {
				// First analog module
                lowDashData.addCluster();
                {
                    for (i = 1; i <= 8; i++)
                        lowDashData.addFloat((float)
								AnalogModule.getInstance(1).getAverageVoltage(i));
                }
                lowDashData.finalizeCluster();

				// Second analog module
                lowDashData.addCluster();
                {
                    for (i = 1; i <= 8; i++)
                        lowDashData.addFloat((float)
								AnalogModule.getInstance(2).getAverageVoltage(i));
                }
                lowDashData.finalizeCluster();
            }
            lowDashData.finalizeCluster();

			// Digital modules
            lowDashData.addCluster();
            {
				// First digital module
                lowDashData.addCluster();
                {
                    lowDashData.addCluster();
                    {
                        int module = 4;
                        lowDashData.addByte(
								DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addByte(
								DigitalModule.getInstance(module).getRelayReverse());
                        lowDashData.addShort(
								DigitalModule.getInstance(module).getAllDIO());
                        lowDashData.addShort(
								DigitalModule.getInstance(module).getDIODirection());

						// PWM Data
                        lowDashData.addCluster();
                        {
                            for (i = 1; i <= 10; i++)
                                lowDashData.addByte((byte)
										DigitalModule.getInstance(module).getPWM(i));
                        }
                        lowDashData.finalizeCluster();
                    }
                    lowDashData.finalizeCluster();
                }
                lowDashData.finalizeCluster();

				// Second digital module
                lowDashData.addCluster();
                {
                    lowDashData.addCluster();
                    {
                        int module = 6;
                        lowDashData.addByte(
								DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addByte(
								DigitalModule.getInstance(module).getRelayReverse());
                        lowDashData.addShort(
								DigitalModule.getInstance(module).getAllDIO());
                        lowDashData.addShort(
								DigitalModule.getInstance(module).getDIODirection());

						// PWM Data
                        lowDashData.addCluster();
                        {
                            for (i = 1; i <= 10; i++)
                                lowDashData.addByte((byte)
										DigitalModule.getInstance(module).getPWM(i));
                        }
                        lowDashData.finalizeCluster();
                    }
                    lowDashData.finalizeCluster();
                }
                lowDashData.finalizeCluster();
            }
            lowDashData.finalizeCluster();
			// End of digital modules

            lowDashData.addByte(Solenoid.getAllFromModule(
                    RobotConfiguration.solenoidModule));

            // Boolean indicating whether or not we're disabled (TRUE for
            // disabled)
            lowDashData.addBoolean(isDisabled());

			// This is the cluster added for Judge 2011
			// For displaying data on the plot
			/*lowDashData.addCluster();
			{
				// The first element is time, and the other eight can contain
				// data
				lowDashData.addFloat(matchTime);

				// Commanded and actual wheel speeds (uses all 8 channels)
				for (i = 0; i < dsBuffer.Size(); i++)
				{
					// Handle the exceptions here - it would be a shame to stop
					// working due to an error here
					try
					{
						lowDashData.addFloat(dsBuffer.GetData(i));
					}
					catch (Exception ex)
					{
						System.err.println(ex.toString());
						lowDashData.addFloat((float)0.0);
					}
				}
			}
			lowDashData.finalizeCluster();*/
        /*}
        lowDashData.finalizeCluster();
        lowDashData.commit();
	}*/
}