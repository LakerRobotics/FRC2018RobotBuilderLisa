package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author Rich Topolewski
 * 
 * Used to take the speed calculated from the PID control and pass it to the drive train, 
 * and while passing through the left to right motor powers are tweeked so it get on the desire gyro angle drive straight
 *
 */
public class PIDOutputStraightSimplerFrom2016 implements PIDOutput {
	double m_TargetAngle = 0;
	
   	//TODO bring this 2016 code into 2018 Code base.  this was really good at keeping the robot on a straight line, it fights really hard it you push it to get backon the right gyro angle

	// This is just a simple P control, Proportional control of the line follow
	// if we assume angle is in degrees and if we were off by 20 Degrees then we would want how much correction
	// for example id Kp is 0.025 at 20 degrees we would have 0.5 or half the power toward rotating the robot 
	double Kp = 0d/20d; //0.025;// 
	Gyro m_gyro; 
	

	public PIDOutputStraightSimplerFrom2016(Gyro theGyro) {
//	    SmartDashboard.putString("rotateRobotPIDOutput", "constructor called");
		m_gyro = theGyro;
	}
	
	public void setTargetAngle(double targetAngle) {
		m_TargetAngle = targetAngle;
	}

	@Override
	public void pidWrite(double motorPower) {
	   	double angleReading = m_gyro.getAngle(); // get current heading
	   	double angle = angleReading - m_TargetAngle;
	    double rotationPower = -angle*Kp*(motorPower/Math.abs(motorPower));// the (motorPower/Math.abs(motorPower) applies the sign correctly if goint forward or backwards
	    
	   	//double rotationPower = 0;
	   	//TODO bring this 2016 code into 2018 Code base.  this was really good at keeping the robot on a stright line, it fights really hard it you push it to get backon the right gyro angle
	    
	    //RobotMap.driveTrainRobotDrive21.arcadeDrive(/*moveValue*/ motorPower, /*rotateValue*/ rotationPower); // drive towards heading 0
	    SmartDashboard.putNumber("RobotDriveStraitPIDOoutput Motor Output",motorPower);
	    SmartDashboard.putNumber("RobotDriveStraitPIDOoutput RotationPower", rotationPower);
	}

}

