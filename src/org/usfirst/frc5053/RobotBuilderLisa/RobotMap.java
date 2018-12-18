// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5053.RobotBuilderLisa;


// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static Encoder driveTrainrightDriveEncoder;
    public static Encoder driveTrainleftDriveEncoder;
    public static SpeedController driveTrainleftDrive1PWM;
    public static SpeedController driveTrainleftDrive2PWM;
    public static SpeedControllerGroup driveTrainleftMotors;
    public static SpeedController driveTrainrightDrive2PWM;
    public static SpeedController driveTrainrightDrive1PWM;
    public static SpeedControllerGroup driveTrainrightMotors;
    public static WPI_TalonSRX elevatorTalonSRX1;
    public static Encoder elevatorENCODER_IS_WIRED_TO_TALON_NEED_TO_MANUALLY_WRITE_CODE;
    public static DigitalInput elevatorelevatorLimitHighDIO;
    public static DigitalInput elevatorelevatorLimitLowDIO;
    public static DoubleSolenoid intakeIntakeSolenoid;
    public static SpeedController intakeintakeLeftPWM;
    public static SpeedController intakeintakeRightPWM;
    public static SpeedController intakeRoller;
    public static Solenoid intakeGripper;
    public static DoubleSolenoid catapultleftSolenoid;
    public static DoubleSolenoid catapultrightSolenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();



    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTrainrightDriveEncoder = new Encoder(2, 3, false, EncodingType.k4X);
        LiveWindow.addSensor("DriveTrain", "rightDriveEncoder", driveTrainrightDriveEncoder);
        driveTrainrightDriveEncoder.setDistancePerPulse(1.0);
        driveTrainrightDriveEncoder.setPIDSourceType(PIDSourceType.kRate);
        driveTrainleftDriveEncoder = new Encoder(0, 1, true, EncodingType.k4X);
        LiveWindow.addSensor("DriveTrain", "leftDriveEncoder", driveTrainleftDriveEncoder);
        driveTrainleftDriveEncoder.setDistancePerPulse(1.0);
        driveTrainleftDriveEncoder.setPIDSourceType(PIDSourceType.kRate);
        driveTrainleftDrive1PWM = new Spark(7);
        LiveWindow.addActuator("DriveTrain", "leftDrive1PWM ", (Spark) driveTrainleftDrive1PWM);
        driveTrainleftDrive1PWM.setInverted(true);
        driveTrainleftDrive2PWM = new Spark(8);
        LiveWindow.addActuator("DriveTrain", "leftDrive2PWM ", (Spark) driveTrainleftDrive2PWM);
        driveTrainleftDrive2PWM.setInverted(true);
        driveTrainleftMotors = new SpeedControllerGroup(driveTrainleftDrive1PWM, driveTrainleftDrive2PWM  );
        LiveWindow.addActuator("DriveTrain", "leftMotors", driveTrainleftMotors);
        
        driveTrainrightDrive2PWM = new Talon(4);
        LiveWindow.addActuator("DriveTrain", "rightDrive2PWM ", (Talon) driveTrainrightDrive2PWM);
        driveTrainrightDrive2PWM.setInverted(true);
        driveTrainrightDrive1PWM = new Talon(3);
        LiveWindow.addActuator("DriveTrain", "rightDrive1PWM ", (Talon) driveTrainrightDrive1PWM);
        driveTrainrightDrive1PWM.setInverted(true);
        driveTrainrightMotors = new SpeedControllerGroup(driveTrainrightDrive1PWM, driveTrainrightDrive2PWM  );
        LiveWindow.addActuator("DriveTrain", "rightMotors", driveTrainrightMotors);
        
        elevatorTalonSRX1 = new WPI_TalonSRX(1);
        
        
        elevatorENCODER_IS_WIRED_TO_TALON_NEED_TO_MANUALLY_WRITE_CODE = new Encoder(4, 7, false, EncodingType.k4X);
        LiveWindow.addSensor("Elevator", "ENCODER_IS_WIRED_TO_TALON_NEED_TO_MANUALLY_WRITE_CODE", elevatorENCODER_IS_WIRED_TO_TALON_NEED_TO_MANUALLY_WRITE_CODE);
        elevatorENCODER_IS_WIRED_TO_TALON_NEED_TO_MANUALLY_WRITE_CODE.setDistancePerPulse(1.0);
        elevatorENCODER_IS_WIRED_TO_TALON_NEED_TO_MANUALLY_WRITE_CODE.setPIDSourceType(PIDSourceType.kRate);
        elevatorelevatorLimitHighDIO = new DigitalInput(5);
        LiveWindow.addSensor("Elevator", "elevatorLimitHighDIO", elevatorelevatorLimitHighDIO);
        
        elevatorelevatorLimitLowDIO = new DigitalInput(6);
        LiveWindow.addSensor("Elevator", "elevatorLimitLowDIO ", elevatorelevatorLimitLowDIO);
        
        intakeIntakeSolenoid = new DoubleSolenoid(0, 4, 5);
        LiveWindow.addActuator("Intake", "IntakeSolenoid", intakeIntakeSolenoid);
        
        intakeintakeLeftPWM = new Talon(0);
        LiveWindow.addActuator("Intake", "intakeLeftPWM ", (Talon) intakeintakeLeftPWM);
        intakeintakeLeftPWM.setInverted(false);
        intakeintakeRightPWM = new Talon(1);
        LiveWindow.addActuator("Intake", "intakeRightPWM ", (Talon) intakeintakeRightPWM);
        intakeintakeRightPWM.setInverted(false);
        intakeRoller = new Spark(6);
        LiveWindow.addActuator("Intake", "Roller", (Spark) intakeRoller);
        intakeRoller.setInverted(false);
        intakeGripper = new Solenoid(0, 6);
        LiveWindow.addActuator("Intake", "Gripper", intakeGripper);
        
        catapultleftSolenoid = new DoubleSolenoid(0, 0, 1);
        LiveWindow.addActuator("Catapult", "leftSolenoid", catapultleftSolenoid);
        
        catapultrightSolenoid = new DoubleSolenoid(0, 2, 3);
        LiveWindow.addActuator("Catapult", "rightSolenoid", catapultrightSolenoid);
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        
        driveTrainleftDriveEncoder.setDistancePerPulse(6*Math.PI/1024); //Distance in inches
        driveTrainleftDriveEncoder.setReverseDirection(true); // False
		
        driveTrainrightDriveEncoder.setDistancePerPulse(6*Math.PI/1024); //Distance in inches
		driveTrainrightDriveEncoder.setReverseDirection(true);
		

		elevatorENCODER_IS_WIRED_TO_TALON_NEED_TO_MANUALLY_WRITE_CODE.setDistancePerPulse(1/360);
		elevatorENCODER_IS_WIRED_TO_TALON_NEED_TO_MANUALLY_WRITE_CODE.setMaxPeriod(1.0);
		
		//m_ElevatorLimitHigh = new DigitalInput(elevatorLimitHighDIO);//What are these for
		//m_ElevatorLimitLow = new DigitalInput(elevatorLimitLowDIO);//What are these for
    }
}
