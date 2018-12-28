package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;

import org.usfirst.frc5053.RobotBuilderLisa.subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDController;

	import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


 
	/**
	 * 
	 * @author Rich Topolewski
	 * 
	 * Used to take the speed calculated from the PID control and pass it to the drive train, but 
	 * before we passes it to the drivetrain we adjust the power difference between the wheels to drive on an arc of specified Radius
	 * 
	 * There is a separate PID controller that is constantly adjusting the turn power on the wheel to be proper for the current speed of the robot.
	 * 
	 *
	 */
	public class PIDOutputArcMotion implements PIDOutput {
		final double Kp = 0.05;//1/200; // so at denominator off in the spin-Rate (RPMP the power will reach the max
	    final double Ki = 0.000;
	    final double Kd = 0.0;
	    final double MaxRotationPower = 0.5;
		//Gyro gyro = Robot.getRobotSensors().getGyro();
		
		// This is just a simple P control, Proportional control of the arc follow
		// if we assume angle is in degrees and if we were off by 20 Degrees then we would want how much correction
		// for example id Kp is 0.025 at 20 degrees we would have 0.5 or half the power toward rotating the robot 

		private DriveTrainMotionControl m_RobotDrive;
//		private double Kp;
		private PIDSource m_Gyro; 
//		private double m_TargetAngle;
		private double m_RotationPower;
		private double m_ForwardPower;
		private double m_ArcRadius;
		PIDController m_ArcRotationSpeedPID;

		/*
		 * arcRadius in Inches
		 */
		public PIDOutputArcMotion(DriveTrainMotionControl drive, PIDSource anglePIDSource, double arcRadius) {
			//SmartDashboard.putString("DriveSpinPIDOutput", "constructor called");
			m_RobotDrive 	= drive;
			m_Gyro 			= anglePIDSource	;
//			m_TargetAngle 	= Double.MAX_VALUE;
			m_ArcRadius 	= arcRadius;
			
//			Kp 				= 0d/20d; //0.025;//
//			m_TargetAngle 	= 0.0d;
			m_RotationPower = 0.0d;
			m_ForwardPower  = 0.0d;
			
//			double slowRotation 					= m_TargetAngle + 90;
			WrapArcPIDOutput wrappedArcPIDOutput 	=  new WrapArcPIDOutput(this);
			
			m_ArcRotationSpeedPID = createArcPIDController(m_Gyro, wrappedArcPIDOutput);
			
			//WrapRotationPIDInput  wrapRotationPIDInput = new WrapRotationPIDOutput(rotationPID, (PIDSource) m_gyro);
		}

		protected synchronized double getRotationPower() {
			return m_RotationPower;
		}


		protected synchronized void setRotationPower(double rotationPower) {
			m_RotationPower = rotationPower;
		}
		
		@Override
		public synchronized void pidWrite(double forwardPower) 
		{
		    //rotationPower
		   	//double rotationPower = 0;
		   	//RobotMap.driveTrainRobotDrive21.arcadeDrive(/*moveValue*/ motorPower, /*rotateValue*/ rotationPower);
			
			//store it away so rotation can be set appropriately
			m_ForwardPower = forwardPower;			
					 
			//Adjust the target rotation speed to correct amount so will trace arch of Radius R.
			m_ArcRotationSpeedPID.setSetpoint(getTargetRotationalSpeed());
			
			SmartDashboard.putNumber("Arc - Target Rotational Speed", m_ArcRotationSpeedPID.getSetpoint());
	    	
	    	//Debugging m_RotationPower = 0;
		    double leftPower; 
	    	double rightPower;
	    	
	    	// Reduce forward power in case when turn would require more then 100% power.
	    	// This can get full and even turning effect
	    	// also may help if quickly reduce from full throttle, to avoid a jerk in the rotation as the PID would convert from 1/2 to all all rotation power
	    	if (forwardPower + m_RotationPower > 1.0){
	    		forwardPower = 1 - m_RotationPower;
	    	}
	    	
	    	SmartDashboard.putNumber("Arc - Rotational Power", m_RotationPower);
	    	SmartDashboard.putNumber("Arc - Forward Power", forwardPower);
	    	
	    	leftPower = m_ForwardPower + m_RotationPower;
	    	rightPower = m_ForwardPower - m_RotationPower;
	    	
	    	SmartDashboard.putNumber("Arc - Left Power Output", leftPower);
	    	SmartDashboard.putNumber("Arc - Right Power Output", rightPower);
	    	

	    	
	    	m_RobotDrive.tankDrive(-leftPower, -rightPower );

		}
		
		private double getTargetRotationalSpeed(){
			boolean debug = true;
			//calculate the correct rotation speed based on the current speed of the robot.
			//arched-turn for a robot in a big circle of radius R and 
			//  it seems the the rate of angler change just needs to be proportional to the speed,
			//  to get a target R circle:
			// RateAngularChange = 360*Speed/2pi*R,
			//		 where: Speed is the speed of the robot in in/sec
			//		 pi is the constant pi
			//		 R is the Radius of the turn path we want the robot to take in inches
			//       360 is number of degrees in full circle.
			double speed = m_RobotDrive.GetAverageSpeed();
//			if (debug) print
			double angularChangeSpeed = (speed * 360)/(2 * Math.PI * m_ArcRadius);
			return angularChangeSpeed;
		}	
		
		public  PIDController createArcPIDController(PIDSource rotationInput, PIDOutput rotationPowerOutput) {
			


		    
		    // rotation speed proportional to the average speed of the wheels
		    // Okay this had been adjusting the speedOfRotation based on how far off they were instead it needs to be adjusted based
		    // on the speed of the robot
		    
			double targetSpin = getTargetRotationalSpeed();	        
	        
	        
			PIDController localRotationSpeedPID = new PIDController(Kp,Ki,Kd, rotationInput,rotationPowerOutput);
		    localRotationSpeedPID.setSetpoint(targetSpin);		    
	        localRotationSpeedPID.setOutputRange(-MaxRotationPower, MaxRotationPower);
	        localRotationSpeedPID.enable();
	        
		    return localRotationSpeedPID;
		}
		
		
	    public double getForwardPower() {
			return m_ForwardPower;
		}
	    
	    public void disableRotationPIDController(){
	    	m_ArcRotationSpeedPID.disable();
	    	//m_RotationController.free();
	    }



	    
		private class WrapArcPIDOutput implements PIDOutput {

	        private PIDOutputArcMotion m_RotationPowerDestination;

	        //Constructor
	        public WrapArcPIDOutput(PIDOutputArcMotion rotationPowerDesintation) {
	            if (rotationPowerDesintation == null){
	                throw new NullPointerException("Given rotationPowerDestination was null");
	            }
	            else{
	            	m_RotationPowerDestination = rotationPowerDesintation;            	
	            }
	        }

			@Override
			public void pidWrite(double rotationPower) {
				this.m_RotationPowerDestination.setRotationPower(rotationPower);
			}
	    }
	}