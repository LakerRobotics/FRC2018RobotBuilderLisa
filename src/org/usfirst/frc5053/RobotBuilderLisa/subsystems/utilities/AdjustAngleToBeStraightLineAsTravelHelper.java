package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author richard.topolewski
 *
 * Note this object is way-overkill, could just put a constant in for the calling routine's place but it allows to test if putting this in place
 *      that everything still works then it will be other one that actaully adjust the angle to do a circle that will be actually useful
 */
public class AdjustAngleToBeStraightLineAsTravelHelper extends AdjustAngleAsTravelHelper {
	double m_targetFixedAngle        = 0.0d; // Just holds and repeats it back,
	/**
     * This helper class just takes a target distance and will provide a Angle for the provided distance (it is, in this case, just a constant)
     */
    public AdjustAngleToBeStraightLineAsTravelHelper(double targetFixedAngle){
    	super();	
    	m_targetFixedAngle = targetFixedAngle;
    }
	@Override
	public double getTargetAngle(double currentMeasuredDistance) {
		return this.m_targetFixedAngle;
	}
    


    
 
}
