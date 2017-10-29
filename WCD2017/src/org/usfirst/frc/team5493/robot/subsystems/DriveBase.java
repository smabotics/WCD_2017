package org.usfirst.frc.team5493.robot.subsystems;

import org.usfirst.frc.team5493.robot.RobotMap;
import org.usfirst.frc.team5493.robot.commands.DriveTankJoystick;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

// The DriveBase subsystem includes the sensors and actuators directly
// attached to the drive base.  These include four drive motors, a left 
// and right encoder, a gyro, and ultrasonic rangefinder

public class DriveBase extends Subsystem {

	double wheelDiameter = 4.0;
	double pulsesPerRevolution = 1440.0;
	double averageDistance;
	public SpeedController leftRearMotor;
    public SpeedController rightRearMotor;
    public SpeedController leftFrontMotor;
    public SpeedController rightFrontMotor;
    public RobotDrive drive;
    public Encoder leftEncoder, rightEncoder;
    public ADXRS450_Gyro gyro;
    //private AnalogInput rangeFinder;
        
    public DriveBase() {
       // Instantiating motor controllers for drive base motors:
       VictorSP leftRearMotor = new VictorSP(RobotMap.L_Rear);
       Victor leftFrontMotor = new Victor(RobotMap.L_Front);
       VictorSP rightRearMotor = new VictorSP(RobotMap.R_Rear);
       Victor rightFrontMotor = new Victor(RobotMap.R_Front);
       
       // Instantiating tank drive base- NOTE:  this is West Coast Drive (6-Wheel, 3 per side, center dropped 1/4")
       // Each side belt driven by single tough-box mini (2 cims/motor controllers)
       // Instantiating Left & Right Encoders (AM3132), Gyro (ADXRS450 in SPI Socket)
       // and Rangefinder (MaxSonar MB1200)
       drive= new RobotDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
       drive.setExpiration(0.1);
       leftEncoder = new Encoder(RobotMap.L_Encoder_A,RobotMap.L_Encoder_B, false, Encoder.EncodingType.k4X);
       rightEncoder = new Encoder(RobotMap.R_Encoder_A,RobotMap.R_Encoder_B,false, Encoder.EncodingType.k4X);
       gyro = new ADXRS450_Gyro();
       //rangeFinder = new AnalogInput(RobotMap.Sonar);
     
   		// Set encoder distance per pulse: 1440 pulses per revolution and
       // 4 inch diameter wheels... Encoder on Wheel Shaft SO 1 wheel revolution
       // is equal to one encoder revolution
       
       leftEncoder.setDistancePerPulse((wheelDiameter*Math.PI)/pulsesPerRevolution);
       rightEncoder.setDistancePerPulse((wheelDiameter*Math.PI)/pulsesPerRevolution);
       
       // Display everything on the LiveWindow:
       LiveWindow.addActuator("Drive Base", "Left Front Motor", leftFrontMotor);
       LiveWindow.addActuator("Drive Base", "Right Front Motor", rightFrontMotor);
       LiveWindow.addActuator("Drive Base", "Left Rear Motor", leftRearMotor);
       LiveWindow.addActuator("Drive Base", "Right Rear Motor", rightRearMotor);
       LiveWindow.addSensor("Drive Base", "Left Encoder", leftEncoder);
       LiveWindow.addSensor("Drive Base", "Right Encoder", rightEncoder);
       //LiveWindow.addSensor("Drive Base", "RangeFinder", rangeFinder);
       LiveWindow.addSensor("Drive Base", "Gyro", gyro);
    }
    
    //When no other command is running, let the operator drive the robot using the joystick controller
    	public void initDefaultCommand() {
		setDefaultCommand(new DriveTankJoystick());
	}
			
	//public void drive(Joystick joy) {
	//	drive(-0.5*joy.getRawAxis(RobotMap.JOYAX_LY), -0.5*joy.getRawAxis(RobotMap.JOYAX_RY));
	//}

	public void drive(double x, double y) {
		drive.tankDrive(x, y);
	}
	
	// Display everything in SmartDashboard:
	public void log()  {
	       SmartDashboard.putNumber("Left Distance", this.leftEncoder.getDistance());
	       SmartDashboard.putNumber("Right Distance", this.rightEncoder.getDistance());
	       //SmartDashboard.putNumber("Left Speed", this.leftEncoder.getRate());
	       //SmartDashboard.putNumber("Right Speed", this.rightEncoder.getRate());
	       SmartDashboard.putNumber("Gyro Angle", this.gyro.getAngle());
	       //SmartDashboard.putNumber("Rangefinder (inches)", rangeFinder.getVoltage());
	}
	
	// Reset the initial gyro and encoder values to zero
   	public void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
	}
   	
   	public void resetGyro() {
   		gyro.reset();
   	}
	
	public double getHeading() {
		return gyro.getAngle();
	}
	
	public double averageDistance() {
		return Math.abs((leftEncoder.getDistance() + rightEncoder.getDistance())/2.0);
	}
}

