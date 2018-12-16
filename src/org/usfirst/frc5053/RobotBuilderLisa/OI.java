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

import org.usfirst.frc5053.RobotBuilderLisa.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc5053.RobotBuilderLisa.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public JoystickButton driverRightTrigger;
    public JoystickButton driverRightBumper;
    public JoystickButton joystickButton1;
    public Joystick driver;
    public Joystick operator;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    public RobotInterfaceMap joysticks; // Laker robotic's helps map to friendly names
    
    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        operator = new Joystick(1);
        
        driver = new Joystick(0);
        
        joystickButton1 = new JoystickButton(driver, 3);
        joystickButton1.whileHeld(new Leftintake());
        driverRightBumper = new JoystickButton(driver, 6);
        driverRightBumper.whenPressed(new CatapultLaunch(0));
        driverRightTrigger = new JoystickButton(driver, 6);
        driverRightTrigger.whenPressed(new CatapultLaunch(20));


        // SmartDashboard Buttons
        SmartDashboard.putData("DriveTeleop", new DriveTeleop());
        SmartDashboard.putData("DriveStraight", new DriveStraight());
        SmartDashboard.putData("DriveTurn", new DriveTurn());
        SmartDashboard.putData("DriveTurnSwing", new DriveTurnSwing());
        SmartDashboard.putData("DriveTurnArc", new DriveTurnArc());
        SmartDashboard.putData("DriveSquare", new DriveSquare());
        SmartDashboard.putData("AutonTest", new AutonTest());
        SmartDashboard.putData("CatapultArmed", new CatapultArmed());
        SmartDashboard.putData("CatapultLaunch: SWITCH_CATAPULT_DELAY", new CatapultLaunch(0.1));
        SmartDashboard.putData("CatapultLaunch: SCALE_CATAPULT_DELAY", new CatapultLaunch(20));
        SmartDashboard.putData("Left intake", new Leftintake());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
 		joysticks = new RobotInterfaceMap(driver  ,RobotInterfaceMap.JoystickType.XBOX,
        		                                   operator,RobotInterfaceMap.JoystickType.JOYSTICK);

    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getDriver() {
        return driver;
    }

    public Joystick getOperator() {
        return operator;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

