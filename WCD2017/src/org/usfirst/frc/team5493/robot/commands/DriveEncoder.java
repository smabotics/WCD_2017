package org.usfirst.frc.team5493.robot.commands;

import org.usfirst.frc.team5493.robot.Robot;
//import org.usfirst.frc.team5493.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class DriveEncoder extends Command {

	double setPoint = 72.0;  // distance to travel (and then stop)
	double adjustedMotorPower;  // adjusted motor power based on difference b/w actual position and setpoint
	double averageDistance;  // the average distance from initial position (actual position)
	double errorSP_AD;  // the error between the actual position and setpoint
	double maxPower = 0.8;  // set arbitrary maximum limit to motor power
	
    public DriveEncoder() {
        requires(Robot.driveBase);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	// reset sensors starting values to zero
    	Robot.driveBase.resetEncoders();
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	averageDistance = Robot.driveBase.averageDistance();
    	errorSP_AD = setPoint - averageDistance;
    	adjustedMotorPower = maxPower*errorSP_AD/setPoint;
    	Robot.driveBase.drive(adjustedMotorPower, adjustedMotorPower);
    	if(errorSP_AD <= 0.0) {
        	Robot.driveBase.drive(0.0,0.0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.driveBase.drive(0.0,0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
