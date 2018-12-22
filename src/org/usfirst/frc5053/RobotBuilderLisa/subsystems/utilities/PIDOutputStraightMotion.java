package org.usfirst.frc5053.RobotBuilderLisa.subsystems.utilities;

//FromRudy import org.usfirst.frc5053.FRC2016Stronghold.MotionControlHelper;
//FromRudy import org.usfirst.frc5053.FRC2016Stronghold.MotionControlPIDController;
//FromRudy import org.usfirst.frc5053.FRC2016Stronghold.RobotMap;
import org.usfirst.frc5053.RobotBuilderLisa.subsystems.DriveTrainMotionControl;

import edu.wpi.first.wpilibj.PIDOutput;

	
	import edu.wpi.first.wpilibj.PIDSource;
	import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

	/**
	 * 
	 * @author Rich Topolewski
	 * 
	 * Used to take the speed calculated from the PID control and pass it to the drive train, 
	 * and also adjust the speed going to the wheels to drive straight
	 *
	 */
	public class PIDOutputStraightMotion implements PIDOutput {
		
		double maxRotationPower = 0.5;

		private DriveTrainMotionControl m_driveTrain;
		private PIDSource m_TurnSource;
		private double m_targetAngle = 0.0d;
		private double rotationPower = 0.0d;
		private MotionControlPIDController m_RotationController;
		

		public PIDOutputStraightMotion(DriveTrainMotionControl drivetrain, PIDSource turnSource, double targetAngle) 
		{
			m_targetAngle = targetAngle;
			m_driveTrain = drivetrain;
			m_TurnSource = turnSource;
			
			double slowRotation = m_targetAngle + 90;// because we use motion control to start somewhere, and go to straight
			WrapRotationPIDOutput wrappedRotationPIDOutput =  new WrapRotationPIDOutput(this);
			
			m_RotationController = createRotationPIDController(m_targetAngle, slowRotation, wrappedRotationPIDOutput);
			
			//WrapRotationPIDInput  wrapRotationPIDInput = new WrapRotationPIDOutput(rotationPID, (PIDSource) m_gyro);
		}

		protected synchronized double getRotationPower() 
		{
			return rotationPower;
		}


		protected synchronized void setRotationPower(double rotationPower) 
		{
			this.rotationPower = rotationPower;
		}


		@Override
		public synchronized void pidWrite(double motorPower) 
		{
		    //rotationPower
		   	//double rotationPower = 0;
		   	//RobotMap.driveTrainRobotDrive21.arcadeDrive(/*moveValue*/ motorPower, /*rotateValue*/ rotationPower); 
		    SmartDashboard.putNumber("RobotDriveStraightPIDOoutput Motor Output",motorPower);
		    SmartDashboard.putNumber("RobotDriveStraightPIDOoutput RotationPower", rotationPower);
		    
	    	double leftPower; 
	    	double rightPower;
	    	
	    	// Reduce joystick power so can get full turning effect, and hopefully avoid a jerk in the rotation
	    	// if quickly taken off full throttle.
	    	if (motorPower + rotationPower > 1.0){
	    		motorPower = 1 - rotationPower;
	    	}
	    	
	    	leftPower = motorPower-rotationPower;
	    	rightPower = motorPower+rotationPower;
	    	
	    	m_driveTrain.tankDrive(-leftPower ,  -rightPower );
	    	
	    	//System.out.println("Left Power: " + leftPower);
	    	//System.out.println("Right power: " + rightPower);

		}
		
		public  MotionControlPIDController createRotationPIDController(double targetAngle, double start, PIDOutput pidOutput) 
		{
			
		    double ramp 	=  30; //degrees
		    double maxspeed = 10.0*(360/60) ; //60/360 converts the first numbers which is in RPM to degrees/second
			
			// This is just a simple P control, Proportional control of the line follow
			// if we assume angle is in degrees and if we were off by 20 Degrees then we would want how much correction
			// for example id Kp is 0.025 at 20 degrees we would have 0.5 or half the power toward rotating the robot 
//			private double Kp = 1d/200d; //0.025;// 
			final double Kp = 0.001; // so at denominator off in the spin-Rate the power will reach the max
		    final double Ki = 0.0001;
		    final double Kd = 0.0;
		 
		    MotionControlPIDController localRotationSpeedPID;

		    AdjustSpeedAsTravelHelper rotationSpeedProfile; 
	        rotationSpeedProfile = new AdjustSpeedAsTravelMotionControlHelper(targetAngle, ramp, maxspeed, start, m_TurnSource, pidOutput);
	        localRotationSpeedPID = new MotionControlPIDController(Kp,Ki,Kd, rotationSpeedProfile );
	        localRotationSpeedPID.setOutputRange(-maxRotationPower, maxRotationPower);
	        localRotationSpeedPID.setPID(Kp, Ki, Kd, 0);
	        localRotationSpeedPID.enable();
		    return localRotationSpeedPID;
		}
		
		
	    public void disableRotationPIDController()
	    {
	    	m_RotationController.disable();
	    	//m_RotationController.free();
	    }
	    
	    private class WrapRotationPIDOutput implements PIDOutput 
	    {

	        private PIDOutputStraightMotion m_RotationPowerDestination;

	        public WrapRotationPIDOutput(PIDOutputStraightMotion rotationPowerDesintation) 
	        {
	            if (rotationPowerDesintation == null) {
	                throw new NullPointerException("Given rotationPowerDestination was null");
	            }
	            else{
	                m_RotationPowerDestination = rotationPowerDesintation;            	
	            }
	        }

			@Override
			public void pidWrite(double rotationPower) 
			{
				this.m_RotationPowerDestination.setRotationPower(-rotationPower); // Inverted because it's stupid
			}

	    }
	    

	}
