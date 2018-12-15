package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;

import org.usfirst.frc5053.RobotBuilderLisa.subsystems.DriveTrainLaker;

import edu.wpi.first.wpilibj.PIDOutput;

public class TurnPIDOutput implements PIDOutput {
	
	private DriveTrainLaker m_DriveTrain;
	private double POWER = 0.25;
	
	public TurnPIDOutput(DriveTrainLaker driveTrain) {
		m_DriveTrain = driveTrain;
	}
	
	public void pidWrite(double rotationPower) {
		//Left | Right
		if(rotationPower < 0)
			rotationPower = rotationPower - POWER;
		if(rotationPower > 0)
			rotationPower = rotationPower + POWER;
		else
			rotationPower = 0;
		
		m_DriveTrain.tankDrive(-rotationPower, rotationPower);
	}
}