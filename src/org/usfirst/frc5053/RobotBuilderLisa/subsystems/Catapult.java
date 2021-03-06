// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5053.RobotBuilderLisa.subsystems;

import org.usfirst.frc5053.RobotBuilderLisa.RobotMap;
import org.usfirst.frc5053.RobotBuilderLisa.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.DoubleSolenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Catapult extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final DoubleSolenoid leftSolenoid = RobotMap.catapultleftSolenoid;
    private final DoubleSolenoid rightSolenoid = RobotMap.catapultrightSolenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new CatapultArmed());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
	/**
	 *  pass in fractions of a seconds to throw with the piston before reversing 
	 *  
	 */
	public void lauchTimeLimited(double secondsTillReverse ) {
		long milliSecondsTillReverse = (long) (secondsTillReverse*1000);
		int extraNanoSecondTillReverse = (int) (secondsTillReverse*1000*1000000 - milliSecondsTillReverse*1000000);

		//Create a new seperate thread that is just wanting to start running (but can't until we tell it to start) 
		Thread catapultTimedThrowThread = new Thread(() -> {
        	try {
        		// just note the time, so we can see how long we actually did slep for
        		long nanoStart = System.nanoTime();
        		
        		// start throwing the Cube
        		leftSolenoid.set(Value.kForward);
        		rightSolenoid.set(Value.kForward);
        		
        		// wait a short amount of time (this way we can stop short of full throw to make a short throw)
        		// (NOTE: we are asking the computer for nanoseconds accuracy (0.000001 seconds), it will do its best to sleep for the request amount of time but we are asking for more accuracy then it can do.)
        		Thread.sleep(milliSecondsTillReverse, extraNanoSecondTillReverse);//Note goo thing we are on a sperate thread or else the robot would be non-responsive if we put the main thread to sleep
 
        		// Okay were done waiting, now stop throwing by reversing the solinoids to retract
        		leftSolenoid.set(Value.kReverse);
        		rightSolenoid.set(Value.kReverse);
        		
        		// now we can check how long it actually was throwing (NOTE: any variance is usually insignificant and provides for all practical purpose the same throw)
				long nanoEnd = System.nanoTime();
				
	     		double howLongActualFired= (double) (nanoEnd-nanoStart)/(double)1000000000;
        		
        		System.out.println("Catapult.LaunchTimeLimited thread: Wanted to fire for "+(double)secondsTillReverse+" seconds; Catapult actually Fired for "+howLongActualFired+ " seconds.  Error was " +(howLongActualFired -secondsTillReverse)+ " seconds.");
        		
        	} catch (InterruptedException e) {
        		// TODO Auto-generated catch block
        		e.printStackTrace();
        	}
        });
        
        // start the thread that will run the above code totally independently of the normal robot thread
        catapultTimedThrowThread.start();

	}
	
    @Override
    public void periodic() {
        // Put code here to be run every loop
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public void arm() 
	{
		leftSolenoid.set(Value.kReverse);
		rightSolenoid.set(Value.kReverse);
	}
	public Value getState()
	{
		return leftSolenoid.get();
	}

}

