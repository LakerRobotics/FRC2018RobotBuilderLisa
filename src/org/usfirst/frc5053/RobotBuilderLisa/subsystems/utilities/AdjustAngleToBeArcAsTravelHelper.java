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
public class AdjustAngleToBeArcAsTravelHelper extends AdjustAngleAsTravelHelper {

	protected double m_startDistance;
	protected double m_startAngle;
	protected double m_angleChangePerDistance;

	/**
     * This helper class just takes a target distance and will provide a motion control speed to get to that target
     * @param aTargetDistance   Where we are trying to get to
     * @param aRampUpRampDownDistance  Allows control of how fast we accelerate and decelerate
     * @param aRunningSpeed   the speed we want to travel most of the time, except for ramp up and ramp down
     * @param aInitialMeasuredDistance  so we know where we started from for the ramp up
     */
    public AdjustAngleToBeArcAsTravelHelper(double startDistance, double startAngle, double endAngle, double radius){
    	//store away startDistance and startAngle to be used later when asked what angle should be;
    	m_startDistance = startDistance;
    	m_startAngle = startAngle;
    	
    	// Calculate the rate of change of the angle as we travel along the path

    	double distanceFullCircle = radius * 2 * Math.PI;    	

    	double angleChange = endAngle - startAngle;

    	double percentOfCircleWeAreTraveling = angleChange/360;

    	double distanceTraveling = distanceFullCircle * percentOfCircleWeAreTraveling;
    	
    	m_angleChangePerDistance = angleChange/distanceTraveling;
    	

    	}


	@Override
	public double getTargetAngle(double currentMeasuredDistance) {
		double distanceTraveledSoFar = currentMeasuredDistance - m_startDistance;
		
		// calculate what the angle should be 
		double angleWeWantAtThisDistance = m_startAngle +
				distanceTraveledSoFar * m_angleChangePerDistance; // its a line mx + b; m=angleChangePerDistance x=distanceTravelSoFar b=startAngle
		return angleWeWantAtThisDistance;
	}
    


    
 
}
