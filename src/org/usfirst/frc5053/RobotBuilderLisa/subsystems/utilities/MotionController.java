package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;

import org.usfirst.frc5053.RobotBuilderLisa.subsystems.DriveTrainMotionControl;
import org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities.AdjustSpeedAsTravelMotionControlHelper;
import org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities.MotionControlPIDController;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionController 
{
	private DriveTrainMotionControl m_DriveTrain;
	private AdjustSpeedAsTravelHelper m_AdustsSpeedAsTravelStraightHelper;
	private AdjustSpeedAsTravelMotionControlHelper m_AdjustRpmAsTurnHelper;
	private AdjustSpeedAsTravelHelper m_AdjustSpeedAsTravelArcHelper;
	
	private MotionControlPIDController m_StraightDistancePIDController;
	private PIDOutputStraightMotion    m_StraightRotationPIDOutput;
	
	private MotionControlPIDController m_TurnPIDController;
	
	private MotionControlPIDController m_ArcDistancePIDController;
	private PIDOutputArcMotion         m_ArcRotationPIDOutput;
	
	private double m_targetDistance;
	private double m_targetAngle;
	private double m_StraightTolerance;
	private double m_TurnTolerance;
	private double m_AngularVelocityTolerance;
	private boolean m_PIDEnabled;
	
	private final double TurnKp = 0.0025;
	private final double TurnKi = 0.0020;
	private final double TurnKd = 0.0;
	private final double TurnMaxPower = 0.75;
	
	private final double SwingKp = 0.07;
	private final double SwingKi = 0.0;
	private final double SwingKd = 0.0;

	
	private final double StraightKp = 0.001;
	private final double StraightKi = 0.0;
	private final double StraightKd = 0.0;
	private final double StraightMaxPower = 1.0;

	private final double ArcKp = 0.002;
	private final double ArcKi = 0.001;
	private final double ArcKd = 0.0;
	
	PIDSource m_LineSource;
	PIDSource m_TurnSource;
	
	
	
	public MotionController(DriveTrainMotionControl driveTrainMotionControl, PIDSource distanceSource, PIDSource turnSource)
	{
		m_DriveTrain = driveTrainMotionControl;
		m_LineSource = distanceSource;
		m_TurnSource = turnSource;
		
		m_StraightDistancePIDController = null;
		m_StraightRotationPIDOutput = null;
//		m_ControlledAngleDrivePIDOutput = null;
		m_TurnPIDController = null;
		
		
		m_targetDistance = 0;
		m_targetAngle = 0;
		m_StraightTolerance = 0.5;
		m_TurnTolerance = 0.5;
		m_AngularVelocityTolerance = 15;
		m_PIDEnabled = false;
		
	}
	
	public boolean ExecuteStraightMotion(double distance, double maxspeed, double ramp) {
		double targetAngle = m_DriveTrain.GetAngle();
		return ExecuteStraightMotionProvideAngle( distance,  maxspeed,  ramp, targetAngle);
	}

	public boolean ExecuteControlledAngleDriveMotion(double distance, double maxspeed, double ramp, double targetAngle) {
		return ExecuteStraightMotionProvideAngle( distance,  maxspeed,  ramp, targetAngle);
	}
	
	private boolean ExecuteStraightMotionProvideAngle(double distance, double maxspeed, double ramp, double targetAngle)
	{
		if (!m_PIDEnabled)
		{
			m_targetAngle =  targetAngle;
			m_targetDistance = distance;
			m_DriveTrain.ResetEncoders();
			
			double start = 0;
			
			double convertedDistance = distance;	// Inches
			double convertedSpeed = maxspeed * 12; 	// Converted from Feet/Second to Inches/Second
			double convertedRamp = ramp;			// Inches/Second
			
			if (!(Math.abs(m_DriveTrain.GetLeftDistance()) > Math.abs(m_targetDistance)))
			{
				//Instantiates a new AdjustSpeedAsTravelMotionControlHelper() object for the driveStraightDistance we are going to traverse
				m_StraightRotationPIDOutput = new PIDOutputStraightMotion(m_DriveTrain, m_TurnSource, m_targetAngle);
				m_AdustsSpeedAsTravelStraightHelper = new AdjustSpeedAsTravelMotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, m_LineSource, m_StraightRotationPIDOutput);
				
				//Instantiates a new MotionControlPIDController() object for the new drive segment using the previous MotionControlHelper()
				m_StraightDistancePIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_AdustsSpeedAsTravelStraightHelper);
				m_StraightDistancePIDController.setAbsoluteTolerance(m_StraightTolerance);
				m_StraightDistancePIDController.setOutputRange(-StraightMaxPower, StraightMaxPower);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_StraightDistancePIDController.enable();
				m_PIDEnabled = true;
				return true;
			}
			return false;
		}
		return true;
	}
	
	public boolean ExecuteTurnMotion(double turnAngle)
	{
		//TODO to really have it turn on a Dime should monitor left to right wheel and make sure adding them goes to Zero
		// create a forward motion PID control on that then you can get precise turning.
		if (!m_PIDEnabled)
		{
			m_DriveTrain.ResetGyro();
			
			//TODO Magic numbers need fixing
			//TODO What are the units?
			double maxRPM = 15/*30*/;			// Rotations/Minute
			double ramp = 30/* 3.5 * maxRPM*/;	// I guess its also rotations per minute?
			
			double maxSpeed = maxRPM * 6; // 360 Degrees/60 seconds to convert RPM to speed or degrees per second
			double start = m_DriveTrain.GetAngle();
			m_targetAngle = turnAngle + start;
			
			if (!(Math.abs(m_DriveTrain.GetAngle()-m_targetAngle) < m_TurnTolerance))
			{
				
				//Instantiates a new MotionControlHelper() object for the new turn segment
				m_AdjustRpmAsTurnHelper = new AdjustSpeedAsTravelMotionControlHelper(m_targetAngle, ramp, maxSpeed, start, m_TurnSource, new PIDOutputDriveTurn(m_DriveTrain));
//DontThinkNeeded				m_TurnControl.setTargetDistance(m_targetAngle);
				
				//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
				m_TurnPIDController = new MotionControlPIDController(TurnKp, TurnKi, TurnKd, m_AdjustRpmAsTurnHelper);
				m_TurnPIDController.setOutputRange(-TurnMaxPower, TurnMaxPower);
				
				//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
				m_TurnPIDController.enable();	
				m_PIDEnabled = true;
				
				return true;
			}
			return false;
		}
		return true;
	}
	


	
	/**
	 * 
	 * @param distance  to travel in inches
	 * @param maxSpeed  in ft/sec
	 * @param ramp      in inches
	 * @param radiusOfArc  The radius of the arc travel path of the robot in inches
	 * @return true if it has completed the arc path
	 */
	public boolean ExecuteArcMotion(double distance, double maxSpeed, double ramp, double radiusOfArc)
	{
		m_targetDistance = distance;
		m_DriveTrain.ResetEncoders();
		
		double start = 0;

		
		if (!isPIDEnabled())
		{
			double convertedDistance = distance; 	// In inches
			double convertedSpeed = maxSpeed * 12; 	// convert from feet to inches/second
			double convertedRamp = ramp; 			// in inches
			

//			//Instantiates a new MotionControlHelper() object for the new Arch segment
//			//Instantiates a new AdjustSpeedAsTravelMotionControlHelper() object for the driveStraightDistance we are going to traverse
//			m_StraightRotationPIDOutput = new PIDOutputStraightMotion(m_DriveTrain, m_TurnSource, m_targetAngle);
//			m_AdustsSpeedAsTravelStraightHelper = new AdjustSpeedAsTravelMotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, m_StraightSource, m_StraightRotationPIDOutput);
			// motionControlForwardSpeed
			m_ArcRotationPIDOutput         = new PIDOutputArcMotion(m_DriveTrain, m_TurnSource, radiusOfArc/12);
			m_AdjustSpeedAsTravelArcHelper = new AdjustSpeedAsTravelMotionControlHelper(convertedDistance, convertedRamp, convertedSpeed, start, m_LineSource, m_ArcRotationPIDOutput);
			
//			//Instantiates a new MotionControlPIDController() object for the new drive segment using the previous MotionControlHelper()
//			m_StraightDistancePIDController = new MotionControlPIDController(StraightKp, StraightKi, StraightKd, m_AdustsSpeedAsTravelStraightHelper);
//			m_StraightDistancePIDController.setAbsoluteTolerance(m_StraightTolerance);
//			m_StraightDistancePIDController.setOutputRange(-StraightMaxPower, StraightMaxPower);
//			
			//Instantiates a new MotionControlPIDController() object for the new turn segment using the previous MotionControlHelper()
			m_ArcDistancePIDController = new MotionControlPIDController(ArcKp, ArcKi, ArcKd, m_AdjustSpeedAsTravelArcHelper);
			m_ArcDistancePIDController.setAbsoluteTolerance(m_StraightTolerance);
			m_ArcDistancePIDController.setOutputRange(-1.0, 1.0);
			
//			//Turns the MotionControlPID ON and it will continue to execute by itself until told otherwise.
//			m_StraightDistancePIDController.enable();
//			m_PIDEnabled = true;
			//Turns the MotionControlPID ON and it will continue to execute on a seperate thread by itself until told otherwise.
			m_ArcDistancePIDController.enable();
			return true;
		}
		return true;
	}
	
	public boolean isStraightMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		SmartDashboard.putNumber("Distance Left", m_DriveTrain.GetLeftDistance());
		SmartDashboard.putNumber("Target distance", m_targetDistance);
		SmartDashboard.putNumber("Straight Tolerance", m_StraightTolerance);
		
		//TODO Verify this tolerance works... it should...
		SmartDashboard.putNumber("Average Distance", m_DriveTrain.GetAverageDistance());
		SmartDashboard.putNumber("Target", Math.abs(m_targetDistance - m_StraightTolerance));
		if (Math.abs(m_DriveTrain.GetLeftDistance()) >= Math.abs(m_targetDistance - m_StraightTolerance))
		{
			//Always tripped
			if(m_StraightDistancePIDController != null) {
				m_StraightDistancePIDController.disable();
				m_StraightRotationPIDOutput.disableRotationPIDController();
			}
			m_DriveTrain.ArcadeDrive(0, 0);
			m_PIDEnabled = false;
			return true;
		}
		return false;
	}
	
	public boolean isTurnMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		if (Math.abs(m_DriveTrain.GetAngle()-m_targetAngle) < m_TurnTolerance && Math.abs(m_DriveTrain.getAngularVelocity()) < m_AngularVelocityTolerance)
		{
			m_TurnPIDController.disable();
			m_DriveTrain.ArcadeDrive(0, 0);
			m_PIDEnabled = false;
			return true;
		}
		return false;
	}
	
	public boolean isArcMotionFinished()
	{
		/*
		 * Called while waiting for the MotionControlPID to finish. The PID will be disabled when the end condition is met, and
		 * the return value indicates you can proceed to the next step.
		 * */
		SmartDashboard.putNumber("Distance Left", m_DriveTrain.GetLeftDistance());
		SmartDashboard.putNumber("Target distance", m_targetDistance);
		SmartDashboard.putNumber("Straight Tolerance", m_StraightTolerance);
		
		//TODO Verify this tolerance works... it should...
		if (Math.abs(m_DriveTrain.GetAverageDistance() - m_targetDistance) <= Math.abs(m_StraightTolerance))
		{
			//Always tripped
			m_ArcDistancePIDController.disable();
			
//Dont stop, let motion flow to next if desired			m_DriveTrain.ArcadeDrive(0, 0);
			m_PIDEnabled = false;
			return true;
		}
		return false;
	}

	public boolean isPIDEnabled()
	{
		return m_PIDEnabled;
	}
	
	public void DisablePIDControls()
	{
		if(m_TurnPIDController != null)
		{
			m_TurnPIDController.disable();
		}
		
		if(m_StraightDistancePIDController != null)
		{
			m_StraightDistancePIDController.disable();
			m_StraightRotationPIDOutput.disableRotationPIDController();
		}
		
		if(m_ArcDistancePIDController != null) {
			m_ArcDistancePIDController.disable();
			//TODO
		}
		
	}
}
