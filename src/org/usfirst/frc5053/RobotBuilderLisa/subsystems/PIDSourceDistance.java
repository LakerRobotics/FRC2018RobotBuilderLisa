package org.usfirst.frc5053.RobotBuilderLisa.subsystems;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/*
 * This is to wrap the drive train to provide the wheel encoders as PIDInput sources
 */
public class PIDSourceDistance implements PIDSource {

	DriveTrainMotionControl m_driveTrainMotionControl;
	boolean returnDistance = true;
	
	public PIDSourceDistance(DriveTrainMotionControl the_driveTrainMotionControl) {
		m_driveTrainMotionControl = the_driveTrainMotionControl;
	}

	
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		if(pidSource == PIDSourceType.kDisplacement) {
			returnDistance = true;
		}
		else {
			returnDistance = false; // so return speed
		}
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		if(returnDistance) {
			return PIDSourceType.kDisplacement;
		}else {
			return PIDSourceType.kRate;
		}		
	}

	@Override
	public double pidGet() {
		if(returnDistance) {
			return m_driveTrainMotionControl.GetAverageDistance();
		}
		else {
			return m_driveTrainMotionControl.GetAverageSpeed();			
		}
	}

}
