package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;
import org.usfirst.frc5053.RobotBuilderLisa.subsystems.DriveTrainLaker;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class DistancePIDWrapper implements PIDOutput, PIDSource {
	
	private DriveTrainLaker m_drivetrain;
	public DistancePIDWrapper(DriveTrainLaker drivetrain)
	{
		m_drivetrain = drivetrain;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double pidGet() {
		return (m_drivetrain.GetAverageDistance());
	}

	@Override
	public void setPIDSourceType(PIDSourceType arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void pidWrite(double output) {
		m_drivetrain.SetSpeed(output*.5);
	}

}
