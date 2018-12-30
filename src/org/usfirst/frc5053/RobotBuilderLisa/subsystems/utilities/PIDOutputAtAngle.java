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
	 * and also adjust the speed going to the wheels to drive at a given angle, i.e. straight or changeing i.e. arc turn
	 *
	 */
	public class PIDOutputAtAngle implements PIDOutput {
		
		double maxRotationPower = 1;

		private DriveTrainMotionControl m_driveTrain;
		private PIDSource m_TurnSource;
//		private double m_targetAngle = 0.0d;
		private double rotationPower = 0.0d;
		private MotionControlPIDController m_RotationController;
		
		private AdjustAngleAsTravelHelper m_AdjustAngleAsTravelHelper;
		

		public PIDOutputAtAngle(DriveTrainMotionControl drivetrain, PIDSource turnSource, AdjustAngleAsTravelHelper adjustAngleAsTravelHelper) 
		{
//			m_targetAngle = targetAngle;
			m_AdjustAngleAsTravelHelper = adjustAngleAsTravelHelper;
			m_driveTrain = drivetrain;
			m_TurnSource = turnSource;

			double target_Angle = m_AdjustAngleAsTravelHelper.getTargetAngle(m_driveTrain.GetAverageDistance());
			double slowRotation = target_Angle + 90;// because we use motion control, it requires a start location so it will move to this target angle, so we just need some fictituos place we started (we just grabbed 90 
			//TODO once this is proven to work shoudl really switch this to infiity
			//double slowRotation = Double.MAX_VALUE;
			WrapRotationPIDOutput wrappedRotationPIDOutput =  new WrapRotationPIDOutput(this);
			
			m_RotationController = createRotationPIDController(target_Angle, slowRotation, wrappedRotationPIDOutput);
			
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
		
		/*
		 * Not this should be called before rotational power is set, it is called during pidWrite() in this object
		 *    which will work OK, but there will be time between calls to PID controller say 20msec (0.02 seconds) 
		 *    so angle will always be a little off but if going really fast 10ft/sec then in that 20ms we could have
		 *    traveled 10ft/(1/0.02)= 10/50 = 1/5 of a foot or 12in/5 = 2.4in, be it that the angle should not be much differant if traveling that 
		 *    but if the code the calucate the rotaiton power can call ahead that would be prefered.
		 */
		public void recalcAndSetTargetAngle() {
			
			double targetAngle = m_AdjustAngleAsTravelHelper.getTargetAngle(m_driveTrain.GetAverageDistance());
			
			this.m_RotationController.setSetpoint(targetAngle);
			
		}


		@Override
		public synchronized void pidWrite(double motorPower) 
		{
			recalcAndSetTargetAngle();// see note above in this methods defintion 
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
		    double maxspeed = 20.0*(360/60) ; //60/360 converts the first numbers which is in RPM to degrees/second
			
			// This is just a simple P control, Proportional control of the line follow
			// if we assume angle is in degrees and if we were off by 20 Degrees then we would want how much correction
			// for example id Kp is 0.025 at 20 degrees we would have 0.5 or half the power toward rotating the robot 
//			private double Kp = 1d/200d; //0.025;// 
		    //TODO change Kp to 1/20 or 0.025 so it is better at staying on the line.
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

	        private PIDOutputAtAngle m_RotationPowerDestination;

	        public WrapRotationPIDOutput(PIDOutputAtAngle rotationPowerDesintation) 
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
				this.m_RotationPowerDestination.setRotationPower(-rotationPower); // Inverted because if your off in the positive direction then need to bring it back the other way
			}

	    }
	    

	}
