package org.usfirst.frc.team5493.robot.commands;

import org.usfirst.frc.team5493.robot.Robot;
//import org.usfirst.frc.team5493.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class DriveEncoder extends Command {

    public DriveEncoder() {
        requires(Robot.driveBase);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	// reset sensors starting values to zero
    	Robot.driveBase.reset();
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveBase.drive(0.5, 0.5);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	Robot.driveBase.getDistance();
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveBase.drive(0.0,0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
