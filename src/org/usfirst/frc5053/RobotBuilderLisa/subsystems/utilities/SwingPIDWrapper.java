package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;
import org.usfirst.frc5053.RobotBuilderLisa.subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SwingPIDWrapper implements PIDOutput, PIDSource {
	
	private DriveTrainMotionControl m_drivetrain;
	public SwingPIDWrapper(DriveTrainMotionControl drivetrain)
	{
		m_drivetrain = drivetrain;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		return (m_drivetrain.GetAngle());
	}

	@Override
	public void setPIDSourceType(PIDSourceType arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void pidWrite(double output) {
		m_drivetrain.SwingTurn(-output);
	}

}
