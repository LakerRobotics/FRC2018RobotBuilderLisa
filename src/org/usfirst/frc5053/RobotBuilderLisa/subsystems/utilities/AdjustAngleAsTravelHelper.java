package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;



import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class AdjustAngleAsTravelHelper {

	protected PIDOutput m_output;
	protected PIDSource m_source;
	PIDController regularPIDControl;

	public AdjustAngleAsTravelHelper() {
		super();
	}

    abstract public double getTargetAngle(double currentMeasuredDistance);

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
	 * This returns the PIDSource wrapped so when called by the PIDController the AdjustAngleAsTravelHelper can
	 * adjust the Angle rate that the PIDController is trying to achieve
	 * @return
	 */
	public PIDSource getM_source() {
		return new wrapPIDInput(this, m_source);
	}

	/**
	 * Makee sure The PIDSource  is
	 * returning rate to the PIDController
	 */
	public void ensureSourceProvidesRate() {
		
		m_source.setPIDSourceType(PIDSourceType.kRate);
	
	}

	/**
	 * Read the input(i.e. position) and calculate the speed for this position and put that in as the setPoint
	 */
	protected void adjustTargetAngle() throws Exception {
		//using the current measurement get the desired Angle
		
//		ensureSourceProvidesRate();
		double currentAngle = m_source.pidGet();
		
		// get the adjusted target speed, this is provided by the implementation class.
		double targetAngle = getTargetAngle(this.getMeasurment());
		
		SmartDashboard.putNumber("AdjustAngleAsTravelHelper.adjustTargetAngle() Measurement", this.getMeasurment());
		//System.out.println("MotionControlHelper.adjustTargetSpeed targetSpeed="+targetSpeed + "  ActualSpeed="+currentSpeed  + "targetPosition="+ this.m_targetDistance+"    Current Postion="+this.getMeasurment());
//NOTSURE		this.getRegularPIDControl().setSetpoint(targetSpeed);
		
		
//NOTSURE		SmartDashboard.putNumber("AdjustAngleAsTravelHelper.adjustTargetAngle() targetSpeed", targetSpeed);
		
//		ensureSourceProvidesRate();
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

	        private AdjustAngleAsTravelHelper m_AngleHelper;
	        private PIDSource m_source; 

	        public wrapPIDInput(AdjustAngleAsTravelHelper adjustAngleAsTravelHelper, PIDSource source) {
	            if (adjustAngleAsTravelHelper == null) {
	                throw new NullPointerException("Given AdjustAngleAsTravelHelper was null");
	            }
	            else{
	                m_AngleHelper = adjustAngleAsTravelHelper;            	
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
	        	// have the controller set the target Angle,
	        	//TODO have WPI redo the PIDController so the calculate() method is protected so we wouldn't have to do this hack 
				//  if it were protected then we could override calculate() method and allow the target speed to be set ahead of calculation the new PID output
				try{
					m_AngleHelper.adjustTargetAngle();
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