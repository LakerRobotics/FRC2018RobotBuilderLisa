package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;

import org.usfirst.frc5053.RobotBuilderLisa.subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.PIDOutput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDOutputDriveTurn implements PIDOutput {

	protected DriveTrainMotionControl m_DriveTrain;
	
	public PIDOutputDriveTurn(DriveTrainMotionControl driveTrain) {
	    SmartDashboard.putString("DriveSpinPIDOutput", "constructor called");
	    m_DriveTrain = driveTrain;
	}

	@Override
	public void pidWrite(double output) {
		m_DriveTrain.tankDrive(-output,output); 
		//System.out.println("DriveSpinPIDOutput Rotation Motor Output:"+output);
		SmartDashboard.putNumber("DriveSpinPIDOutput Rotation Motor Output",output); 
	}

}