// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5053.RobotBuilderLisa.commands;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc5053.RobotBuilderLisa.Robot;

/**
 *
 */
public class AutonTest extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public AutonTest() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveTrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
//    	Robot.driveTrain.arcadeDrive(50, 0);
    	
    }

    int count = 1;
	double time = 0.01;
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	count = count + 1;
    	//so ever 100 calls (about 2 seconds)
    	if(count % 100 == 0) {
    			time = time + 0.005;
    			
    			// Dont let thread run into the next launch (assumed to be at about 2 second)
    			if (time > 1.7)  time = 1.7;
    			Robot.catapult.lauchTimeLimited(time);
    			
    	}
    	
//    	Robot.driveTrain.arcadeDrive(50, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
