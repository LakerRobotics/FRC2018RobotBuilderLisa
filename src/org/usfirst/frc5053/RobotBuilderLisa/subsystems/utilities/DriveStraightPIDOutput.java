package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;

import org.usfirst.frc5053.RobotBuilderLisa.subsystems.DriveTrainLaker;

import edu.wpi.first.wpilibj.PIDOutput;

public class DriveStraightPIDOutput implements PIDOutput {
	
	private DriveTrainLaker m_DriveTrain;
	
	public DriveStraightPIDOutput(DriveTrainLaker driveTrain) {
		m_DriveTrain = driveTrain;
	}
	
	public void pidWrite(double rotationPower) {
		//Left | Right
		m_DriveTrain.tankDrive(-rotationPower, rotationPower);
	}
}
