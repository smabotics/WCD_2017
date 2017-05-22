package org.usfirst.frc.team5493.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	// Setting up XBOX 360 Controller (port 0 on Drive Station USB connections)
	public final static int DRIVE_JOYSTICK = 0;
	// Variable name indicates button 
	// Number is designated on Drive Station (USB connections)
	public final static int JOYB_A = 1;
	public final static int JOYB_B = 2;
	public final static int JOYB_X = 3;
	public final static int JOYB_Y = 4;
	public final static int JOYB_LB = 5;
	public final static int JOYB_RB = 6;
	// Variable name indicates joystick X or Y Axis
	// Number is designated on Drive Station (USB connections)
	public final static int JOYAX_LX = 0;
	public final static int JOYAX_LY = 1;
	public final static int JOYAX_RX = 4;
	public final static int JOYAX_RY = 5;
	
	// Setting up Motor Controllers for West Coast Drive Base
	// Number is PWM Port on Roborio
	public static int L_Front = 7;   //Victor 888
	public static int R_Front = 2;   //Victor 888
	public static int L_Rear = 8;   //Victor SP
	public static int R_Rear = 1;   //Victor SP

	// Setting up Distance Sensor (MAXSONAR XL-EZ/AE MB1200)
	// Number is ANALOG Port on Roborio
	public static int Sonar = 1;
	
	// Setting up Gyro/Accelerometer (ADXL362/ ADXRS450)
	// Direct-Connect to SPI Socket on Roborio
	
	// Setting Up Encoders (AM3132, installed on Tough-Box Mini Drive Shaft)
	// One encoder rotation is one wheel rotation
	// Number is DIO Port on Roborio
	public static int R_Encoder_A = 2;
	public static int R_Encoder_B = 1;
	public static int L_Encoder_A = 8;
	public static int L_Encoder_B = 7;
	
	
}
