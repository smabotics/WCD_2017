package org.usfirst.frc.team5493.robot.commands;

import org.usfirst.frc.team5493.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class DriveGyro extends Command {

	double gyroAngle;  // actual angle in degrees (from 0 to 360) robot is moving towards
	double angleSetPoint = 180.0;  // I'm thinking this follows a compass, indicating that due East is 180 degrees (i.e. desired direction)
	double errorASP_GA;  // error between actual heading and desired heading in degrees
	double maxAdjust = 0.25;  // factor applied to the error, applied to motor power correction
	double medianPower = 0.5;  // median power applied to drivebase motors
	double motorPowerAdjust;  // motor power correction to median power
	
    public DriveGyro() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveBase);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveBase.resetGyro();
    	setTimeout(2.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	gyroAngle = Robot.driveBase.getHeading();
    	errorASP_GA = angleSetPoint - gyroAngle;
    	motorPowerAdjust = maxAdjust*errorASP_GA;
    	Robot.driveBase.drive(medianPower+motorPowerAdjust, medianPower-motorPowerAdjust);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
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
