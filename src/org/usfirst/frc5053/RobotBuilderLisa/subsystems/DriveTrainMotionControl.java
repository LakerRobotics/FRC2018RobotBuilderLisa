package org.usfirst.frc5053.RobotBuilderLisa.subsystems;



import org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities.MotionController;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder; 
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Drivetrain subsystem that extends the FRC RobotDrive class.
 * 
 *
 */

public class DriveTrainMotionControl extends DifferentialDrive 
{
	private Encoder m_LeftEncoder;
	private Encoder m_RightEncoder;
	
	private MotionController m_MotionController;
	
	private ADXRS450_Gyro m_Gyro;
	
	public boolean isPIDRunning = false;
	
	// Normal PID stuff D:
//	PIDController m_AnglePID;
//	PIDController m_SwingPID;

//	private SwingPIDWrapper m_SwingPIDWrapper;
//	private double m_Speed = 0.0;
//	private final double SWING_TOLERANCE = 1.0;
	
	public DriveTrainMotionControl(SpeedControllerGroup leftMotorGroup, SpeedControllerGroup rightMotorGroup, Encoder leftEncoder, Encoder rightEncoder, ADXRS450_Gyro gyro)
	{
		super(leftMotorGroup, rightMotorGroup);
		
		//m_DriveTrain = new DriveTrain(leftMotorGroup, rightMotorGroup, leftEncoder, rightEncoder, gyro);
		
		m_LeftEncoder = leftEncoder;
		m_RightEncoder = rightEncoder;
		
		m_Gyro = gyro;
		
		PIDSource robotDistanceEncoders = new PIDSourceDistance(this);
		
		m_MotionController = new MotionController(this, robotDistanceEncoders , (PIDSource) m_Gyro);
		

		
//		m_SwingPID.setOutputRange(-0.75, 0.75);
//		m_SwingPID.setAbsoluteTolerance(SWING_TOLERANCE);
	}
	
	
	

	// Drive Straight
	public void DriveDistance(double distance, double maxspeed, double ramp){
		if(!isPIDRunning){
			isPIDRunning = 	m_MotionController.ExecuteStraightMotion(distance, maxspeed, ramp);
		}
	}
	public boolean isStraightPIDFinished(){
		if(m_MotionController.isStraightMotionFinished()){
			isPIDRunning = false;
			return true;
		}
		return false;
	}
	
	public void DriveControlledAngle(double distance, double maxspeed, double ramp, double angle){
		if(!isPIDRunning)
			isPIDRunning = m_MotionController.ExecuteControlledAngleDriveMotion(distance, maxspeed, ramp, angle);
	}
	
	// Turn (piorouette)
	//=====================================
	public void TurnToAngle(double turnAngle){
		if(!isPIDRunning){
			isPIDRunning = m_MotionController.ExecuteTurnMotion(turnAngle);
		}
	}

	public boolean isTurnPIDFinished() {
		if(m_MotionController.isTurnMotionFinished()){
			isPIDRunning = false;
			return true;
		}
		return false;
	}
	
	/*
	 * Drive on Arc
	 * --------------------------------------------------------------------------------------------
	 */
	public void DriveInArc(double distance, double maxspeed, double ramp, double radius){
		if(!isPIDRunning){
			isPIDRunning = m_MotionController.ExecuteArcMotion(distance, maxspeed, ramp, radius);
		}
	}
	public boolean isArcPIDFinished(){
		if(m_MotionController.isArcMotionFinished()){
			isPIDRunning = false;
			return true;
		}
		return false;
	}
	
	
	public void DisablePIDControl(){
		m_MotionController.DisablePIDControls();
	}
	
	public double GetLeftDistance() { 	return m_LeftEncoder.getDistance();}
	public double GetRightDistance(){ 	return m_RightEncoder.getDistance();}
	public double GetAverageDistance(){	return (GetLeftDistance() + GetRightDistance())/2;}
	
	public double GetLeftSpeed(){		return m_LeftEncoder.getRate();}
	public double GetRightSpeed(){    	return m_RightEncoder.getRate();}
	public double GetAverageSpeed(){	return (GetLeftSpeed() + GetRightSpeed())/2;}

	public void ResetEncoders(){
		m_LeftEncoder.reset();
		m_RightEncoder.reset();
	}
	public void ResetGyro() {
		m_Gyro.reset();
	}

	public void ArcadeDrive(double powerForward, double powerRotation){
		this.arcadeDrive(powerForward, -powerRotation);
	}
	
	public double GetAngle(){
		//Store away the gyrotype (e.g. distance or speed, angle or Rev/min)
		PIDSourceType gyroType = m_Gyro.getPIDSourceType();
		
		// force type to be distance, which in this case is an Angle mesurement
		m_Gyro.setPIDSourceType(PIDSourceType.kDisplacement);
		double angle = m_Gyro.getAngle();
		
		//return the Gyro to the original setting
		m_Gyro.setPIDSourceType(gyroType);
		
		// return the angle
		return angle;
	}
	public double getAngularVelocity(){
		return m_Gyro.getRate();
	}
	

	
	
//	public HashMap<String, Double> GetDashboardData() {
//		return null;
//		// TODO Auto-generated method stub
//	}
	
	public void WriteDashboardData() {
		SmartDashboard.putNumber("Gyro Angle", m_Gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate", m_Gyro.getRate());
		
		// Do not change these names they are used for the DS Dashboard
		SmartDashboard.putNumber("leftDriveEncoder", this.GetLeftDistance());
		SmartDashboard.putNumber("rightDriveEncoder", this.GetRightDistance());
		
		SmartDashboard.putNumber("AverageEncoderDistance", this.GetAverageDistance());
		
		SmartDashboard.putNumber("LeftDriveEncoder Rate", this.GetLeftSpeed());
		SmartDashboard.putNumber("RightDriveEncoder Rate", this.GetRightSpeed());
		SmartDashboard.putNumber("Average Rate", this.GetAverageSpeed());
		
	}

}
