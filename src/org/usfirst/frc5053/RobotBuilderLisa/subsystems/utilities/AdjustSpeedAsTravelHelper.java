package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;



import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class AdjustSpeedAsTravelHelper {

	protected PIDOutput m_output;
	protected PIDSource m_source;
	PIDController regularPIDControl;

	public AdjustSpeedAsTravelHelper() {
		super();
	}

    abstract public double getTargetSpeed(double currentMeasuredDistance);

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
		
		// get the adjusted target speed, this is provided by the implementation class.
		double targetSpeed = getTargetSpeed(this.getMeasurment());
		
		SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed Measurement", this.getMeasurment());
		//System.out.println("MotionControlHelper.adjustTargetSpeed targetSpeed="+targetSpeed + "  ActualSpeed="+currentSpeed  + "targetPosition="+ this.m_targetDistance+"    Current Postion="+this.getMeasurment());
		this.getRegularPIDControl().setSetpoint(targetSpeed);
		SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed targetSpeed", targetSpeed);
		
		ensureSourceProvidesRate();
		SmartDashboard.putNumber("MotionControlHelper.adjustTargetSpeed Gyro Rate", m_source.pidGet());
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
	   class wrapPIDInput implements PIDSource {

	        private AdjustSpeedAsTravelHelper m_MCHelper;
	        private PIDSource m_source; 

	        public wrapPIDInput(AdjustSpeedAsTravelHelper motionControlHelper, PIDSource source) {
	            if (motionControlHelper == null) {
	                throw new NullPointerException("Given AdjustSpeedAsTravelHelper was null");
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
	        	// have the controller set the target speed,
	        	//TODO have WPI redo the PIDController so the calculate() method is protected so we wouldn't have to do this hack 
				//  if it were protected then we could override calculate() method and allow the target speed to be set ahead of calculation the new PID output
				try{
					m_MCHelper.adjustTargetSpeed();
				}
				catch (Exception e){
					System.out.println("MotionControl PIDSource BAD, likley not Gyro or Encoder or missing getMeasurement()");
					System.out.println(e);
				}
				// call the real PIDSource
	        	return m_source.pidGet();
	        }

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				m_source.setPIDSourceType(pidSource);
//				System.out.println("ERROR MotionControlHelper.setPIDSourceType() CALL BEING IGNORED because this Motion control controls Rate");
				
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				return m_source.getPIDSourceType();
				//return PIDSourceType.kRate;
				//return m_pidSource;
			}

	    }

}