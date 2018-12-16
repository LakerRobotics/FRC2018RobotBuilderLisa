package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author richard.topolewski
 *
 */
public class StartEndControlerHelper {
	
	boolean debug = true;

    double m_startDistance          = 0.0d; // start position, in distance units for example inches or Degrees of rotation
    double m_startSpeed             = 0.0d; // start speed, in distance units for example inches or Degrees of rotation
    
	double m_endDistance            = 0.0d; // end position, in distance units for example inches or Degrees of rotation
    double m_endSpeed               = 0.0d; // end speed, in distance units for example inches or Degrees of rotation
    
    double m_ramp                   = 0.0d; // the ramp up/down in speed as we travel along from start to end distance
    	
    double m_currentMeasuredDistance  = 0.0d;
    double m_initialMeasuredDistance  = 0.0d;
 
//  	public double percentDeadZoneOverride = 0.5;//enter portion of 1 (e.g. .1 for 10%)
//   	
//   	private final double RAMP_MULTIPLIER = 4.0;
   
	PIDOutput m_output;
	PIDSource m_source;
	
	PIDController regularPIDControl;
	   	
	/**
     * This helper class just takes a start distance and speed, and and end distance and speed 
     *     and will provide a ramping of speed (either up or down)  to get to that end distance and be traveling athte end speed
     * @param aStartDistance   Where we are Starting at (for example what the wheel encoder should currently be reporting) 
     * @param aStartSpeed      The speed we are to start at (note can't be 0, else we wont go anywhwere to start moving)
     * @param aEndDistance     Where we are trying to get to
     * @param aEndSpeed        The speed we are trying to get to
     * @param source           The source information about, i.e. the encoder we are going to be monitoring
     * @param output           The output, i.e. the motor power we are going to be controlling to achieve the start-end speed profile desired
     */
    public StartEndControlerHelper(double aStartDistance, double aStartSpeed, 
    		                       double aEndDistance,   double aEndSpeed,
    		                       PIDSource source, PIDOutput output)
    {
    		
    	m_startDistance          = aStartDistance; // start position, in distance units for example inches or Degrees of rotation
    	m_startSpeed             = aStartSpeed;    // start speed, in distance units for example inches or RevolutionsPerMin
    	    
    	m_endDistance            = aEndDistance; // end postion, in distance units for example inches or Degrees of rotation
    	m_endSpeed               = aEndSpeed; // end speed, in distance units for example inches or Degrees of rotation
    	
    	if(Math.abs(aStartDistance-aEndDistance) < 0.00000001) {
    		System.out.print("StartEndControllerHelper was started with the start and end being nearly the same, to avoid null pointer dont do this ");
    		m_ramp = 0;
    	} 
    	else {
    		m_ramp = m_endSpeed - m_startSpeed / (m_endDistance-m_startDistance);
    	}

    	m_output = output;
    	m_source = source;
    	
    }

    protected PIDController getRegularPIDControl() {
		return regularPIDControl;
	}

	protected void setRegularPIDControl(PIDController regularPIDControl) {
		this.regularPIDControl = regularPIDControl;
	}
	
    public PIDOutput getM_output() {
		return m_output;
	}

    /**
     * This returns the PIDSource wrapped so when called by the PIDController the motionControlHelper can
     * adjust the target rate that the PIDController is trying to achieve
     * @return
     */
	public PIDSource getM_source() {
		return new wrapPIDInput(this, m_source);
	}

	/**
     * Given the currentPosition (i.e. distance) this will return the current target speed 
     * @param currentMeasuredDistance // the current position, in distance units for example inches or Degrees-of-rotation
     * @return
     */
    public double getTargetSpeed(double currentMeasuredDistance){
    	double distanceTraveledSoFar = currentMeasuredDistance - m_startDistance;
    	double targetSpeed = m_startSpeed + m_ramp*distanceTraveledSoFar;
    	return targetSpeed;
    }
    
	/**
	 * The PIDSource we to ensure it is
	 * returning rate to the PIDController
	 */
	public void ensureSourceProvidesRate() {
		m_source.setPIDSourceType(PIDSourceType.kRate);
	}
    
    /**
     * Read the input(i.e. position) and calculate the speed for this position and put that in as the setPoint
     */
    protected void adjustTargetSpeed() throws Exception {
    	//using the current measurement get the desired rate (i.e. speed)
    	
    	ensureSourceProvidesRate();
    	double currentSpeed = m_source.pidGet();
    	
    	double targetSpeed = getTargetSpeed(this.getMeasurment());
    	if(debug) SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed currentSpeed", currentSpeed);
    	//System.out.println("MotionControlHelper.adjustTargetSpeed targetSpeed="+targetSpeed + "  ActualSpeed="+currentSpeed  + "targetPosition="+ this.m_targetDistance+"    Current Postion="+this.getMeasurment());

    	this.getRegularPIDControl().setSetpoint(targetSpeed);
    	if(debug) SmartDashboard.putNumber("StartEndControlerHelper.adjustTargetSpeed targetSpeed", targetSpeed);
    	
    	ensureSourceProvidesRate();
    	if(debug) SmartDashboard.putNumber("StartEndControlHelper.adjustTargetSpeed getSpeed", m_source.pidGet());
    	// now that we have the speed set properly lets call the PID control and have it adjust the PIDInput (e.g. the motor power) to get closer to the desired speed.
    	//TODO need to access the inner class PIDTask and override to call calculatesSetup then then calculate()
       	//super.calculate();
    }

	public double getMeasurment() {
		// Store away the what the Source is to return
		PIDSourceType tempType = m_source.getPIDSourceType();
		// Switch to report on where were at, and get where we are at
		m_source.setPIDSourceType(PIDSourceType.kDisplacement);
		double returnValue =  m_source.pidGet();
		// revert PIDSource back to what it was reporting before (either Rate or Displacement)
		m_source.setPIDSourceType(tempType);
		// Actually return the measurement (i.e. displacement or location)
		return returnValue;
	}	

    
    private class wrapPIDInput implements PIDSource {

        private StartEndControlerHelper m_MCHelper;
        private PIDSource m_source; 

        public wrapPIDInput(StartEndControlerHelper motionControlHelper, PIDSource source) {
            if (motionControlHelper == null) {
                throw new NullPointerException("The given StartEndControlHelper was null");
            }
            else{
                m_MCHelper = motionControlHelper;            	
            }
            
            if (source == null){
                throw new NullPointerException("Given PIDSource was null");
            }
            else{
                m_source = source;
            }
        }
        
		@Override
        public double pidGet(){
        	// have the controller set the target speed, to be going along our ramp from start speed to end speed
			
        	//TODO have WPI redo the PIDController so the calculate() method is protected so we wouldn't have to do this hack 
			//  if it were protected then we could override calculate() method and allow the target speed to be set ahead of calculation the new PID output
			try{
				m_MCHelper.adjustTargetSpeed();
			}
			catch (Exception e){
				System.out.println("MotionControl PIDSource BAD, likley not Gyro or Encoder or missing getMeasurement()");
				System.out.println(e);
			}
			// Now that the target speed has been adjusted baseed on where we are at along our distance now we can give back the current measuement and it will adjust things to the new adjustee Target Speeed
			// call the real PIDSource
        	return m_source.pidGet();
        }

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			m_source.setPIDSourceType(pidSource);
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return m_source.getPIDSourceType();
		}

    }
    
}
