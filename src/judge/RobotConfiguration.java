/*******************************************************************************
* File:  BalanceFilter.java
* Date:  1/10/2012
* Auth:  K. Loux
* Desc:  Kalman filter implementation for estimating state of robot balancing on
*        teetering platform.
*******************************************************************************/

// Declare our package
package judge;

/**
 * Class for storing robot configuration
 * Mostly wiring - defines all connections, slots, channels, etc.
 *
 * @author kloux
 */
public class RobotConfiguration
{
    // Module slot numbers
    public static final int analogInputModule = 1;
    public static final int digitalSideCarModule = 1;
    public static final int solenoidModule = 2;

	public static final double frequency = 50.0;
	public static final double timeStep = 1.0 / frequency;

	/***************************************************************************
	 *                              Outputs
	 **************************************************************************/
	// PWM Outputs
	public static final int leftFrontMotorChannel = 1;
	public static final int rightFrontMotorChannel = 2;
	public static final int leftRearMotorChannel = 3;
	public static final int rightRearMotorChannel = 4;
	public static final int turretMotorChannel = 5;// FIXME:  No longer used, but needed for clean compile
	public static final int pitcherTopMotorChannel = 5;
	public static final int pitcherBottomMotorChannel = 6;
	public static final int bucketMotorChannel = 7;
	public static final int gateServoChannel = 8;
	public static final int bridgeTipperChannel = 9;
	public static final int brushMotorChannel = 10;

	// Digital outputs
	public static final int relayChannel = 1;

	/***************************************************************************
	 *								 Inputs
	 **************************************************************************/
    // Digital sensors
	public static final int bucketUpperLimitChannel = 1;
	public static final int bucketLowerLimitChannel = 2;
	public static final int pitcherTopEncoderAChannel = 3;
	public static final int pitcherTopEncoderBChannel = 4;
	public static final int pitcherBottomEncoderAChannel = 5;
	public static final int pitcherBottomEncoderBChannel = 6;
	public static final int turretEncoderAChannel = 7;
	public static final int turretEncoderBChannel = 8;
    public static final int pressureSwitchChannel = 11;

	// I2C sensors
	public static final int accelerometerChannel = 1;

	// Analog sensors
	public static final int yawGyroChannel = 2;
	public static final int pitchGyroChannel = 3;

	/***************************************************************************
	 *                        Controller Parameters
	 **************************************************************************/
	// Turret
	public static final double turretPKp = 1.0;
	public static final double turretPKi = 1.0;
	public static final int turretPQueueSize = 10;

	public static final double turretVKp = 1.0;
	public static final double turretVKi = 1.0;
	public static final int turretVQueueSize = 10;
	public static final double turretVOmega = 65.0;// [rad/sec]
	public static final double turretVZeta = 1.0;// [-]

	public static final double pitcherKp = 1.0;
	public static final double pitcherKi = 1.0;
	public static final int pitcherQueueSize = 10;
	public static final double pitcherOmega = 65.0;// [rad/sec]
	public static final double pitcherZeta = 1.0;// [-]

	public static final double balanceKp = 1.0;
	public static final double balanceKi = 1.0;
	public static final int balanceQueueSize = 10;

	public static final double yawKp = 15.0;
	public static final double yawKi = 1.0;
	public static final int yawQueueSize = 10;

	/***************************************************************************
	 *                   Hardware Information and Constants
	 **************************************************************************/
	public static final int usrEncoderPulsesPerRevolution = 360;// [-]
	public static final double usrEncoderAnglePerPulse =
			usrEncoderPulsesPerRevolution / 360.0;// [deg]

	public static final double fisherPriceGearboxRatio = 1.0;// [-] FIXME:  Wrong
	public static final double cimpleBoxGearRatio = 5.0;// [-] FIXME:  Wrong

	public static final double drivetrainP80GearboxRatio = 9.0;// [-]
	public static final double pitcherGearboxRatio = 4.0;// [-]

	public static final double fisherPriceMotorMaxSpeed = 20770.0;// [RPM]
	public static final double cimMotorMaxSpeed = 5310.0;// [RPM]
	public static final double windowMotorMaxSpeed = 84.0;// [RPM]
	public static final double baneBotsRS550MaxSpeed = 16000.0;// [RPM]
	public static final double amGearMotorMaxSpeed = 1.0;// [RPM] FIXME:  Wrong

	public static final double gravity = 32.174;// [ft/sec2]

	public static final double cameraVFOV = 30.0 * Math.PI / 180.0;// [rad] FIXME:  Check

	public static final double robotMass = 120.0 / gravity;// [slug]

	public static final double pitcherSystemInertia = 0.003;// [slug-ft2]
	public static final double ballMass = 0.022;// [slug]
	public static final double launchAngle = 53.7 * Math.PI / 180.0;// [rad]
	public static final double launchToTargetHeight = 4.0;// [ft]
	public static final double wheelToBallContactAngle = 26.6 * Math.PI / 180.0;// [rad]
	public static final double pitchingWheelDiameter = 8.0 / 12.0;// [ft]

	public static final double targetHeight = 18.0;// [in]
	public static final double targetWidth = 24.0;// [in]
	public static final double targetThickness = 2.0;// [in]

	public static final double cameraHeight = 0.0;// [ft]

	public static final double gyroSensitivity = 0.007;// [V/deg/sec]

	/***************************************************************************
	 *                     Sensor Variances (std dev ^2)
	 **************************************************************************/
	public static final double gyroAngleVariance = 1.0;// [deg^2] FIXME:  Verify
	public static final double gyroRateVariance = 0.24 * 0.24;// [deg^2/sec^2] FIXME:  Estimated from datasheet
	public static final double accelerometerVariance = 0.01 * 0.01;// [G^2] FIXME:  Estimated from datasheet
	public static final double usrEncoderRateVariance = 12.63 * 12.63;// [deg^2/sec^2]
	public static final double usrEncoderAngleVariance = 0.25 * 0.25;// [deg^2]
	public static final double cameraDistanceVariance = 0.5;// [ft] FIXME:  Verify
	public static final double cmaeraHorizontalAngleVairance = 5.0;// [deg] FIXME:  Verify
	public static final double rangeFinder1080Variance = 1.0;// [in] FIXME:  Verify
	public static final double rangeFinder20150Variance = 2.0;// [in] FIXME:  Verify
}
