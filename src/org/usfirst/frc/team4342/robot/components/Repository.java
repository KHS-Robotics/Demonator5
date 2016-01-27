package org.usfirst.frc.team4342.robot.components;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.api.logging.PdpLogger;
import org.usfirst.frc.team4342.api.logging.RobotConsoleLogger;
import org.usfirst.frc.team4342.api.logging.RobotLogFactory;
import org.usfirst.frc.team4342.api.pdp.PdpInfoExtractor;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

import com.kauailabs.navx.frc.AHRS;

import ernie.logging.loggers.ILogger;
import ernie.logging.loggers.MultiLogger;

/**
 * Poor man's IoC. This code is the first thing that runs in Robot.robotInit() to initialize
 * all of the objects the robot uses to make them all easily accessible. Of course though Java
 * is dumb and doesn't have properties (getters and setters in one line) so we are lazy and just
 * made the instance variables public. Yay Java and encapsulation! #DealWithIt
 */
public class Repository 
{
	private static boolean initialized;
	
	public static ILogger Log;
	public static RobotConsoleLogger ConsoleLog;
	public static MultiLogger Logs;
	
	public static PdpInfoExtractor Pdp;
	
	public static CANTalon FrontRight, FrontLeft, MiddleRight,
							MiddleLeft, RearRight, RearLeft;
	public static CANTalon[] DriveTrain;
	
	public static Joystick DriveStick;
	
	public static AHRS Navx;
	
	public static DigitalInput PressureSwitch;
	public static Relay Compressor;
	public static DoubleSolenoid Shifter;
	public static Solenoid Cylinder;
	
	public static void initializeAll()
	{
		if(!initialized)
		{
			initializeLogs();
			initializeJoysticks();
			initializeDrive();
			initializePneumatics();
		}
		
		initialized = true;
	}
	
	private static void initializeLogs()
	{
		try
		{
			ConsoleLog = RobotLogFactory.createRobotConsoleLogger();
			Log = RobotLogFactory.createAsyncLogger();
			Logs = new MultiLogger(new ILogger[] { ConsoleLog, Log });
		}
		catch(Exception ex)
		{
			ConsoleLog.warning("Failed to initialize log due to a " + ExceptionInfo.getType(ex));
			Logs = new MultiLogger(new ILogger[] { ConsoleLog });
		}
		
		try
		{
			Pdp = new PdpInfoExtractor();
			PdpLogger.start(Pdp, Log, ConsoleLog);
		}
		catch(Exception ex)
		{
			Logs.warning("Failed to start PDP Logger due to a " + ExceptionInfo.getType(ex));
		}
	}
	
	private static void initializeJoysticks()
	{
		try
		{
			DriveStick = new Joystick(0);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize joysticks due to a " + ExceptionInfo.getType(ex), ex);
		}
	}
	
	private static void initializeDrive()
	{
		try
		{
			FrontRight = new CANTalon(0);
			FrontLeft = new CANTalon(15);
			MiddleRight = new CANTalon(1);
			MiddleLeft = new CANTalon(14);
			RearRight = new CANTalon(2);
			RearLeft = new CANTalon(13);
			
			DriveTrain = new CANTalon[] {
				FrontRight,
				FrontLeft,
				MiddleRight,
				MiddleLeft,
				RearRight,
				RearLeft
			};
			
			for(CANTalon talon : DriveTrain)
			{
				talon.configEncoderCodesPerRev(512);
				talon.enableBrakeMode(true);
				talon.enable();
			}
			
			Navx = new AHRS(SPI.Port.kMXP, (byte)50);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize drive due to a " + ExceptionInfo.getType(ex), ex);
		}
	}
	
	private static void initializePneumatics()
	{
		try
		{
			PressureSwitch = new DigitalInput(1);
			Compressor = new Relay(1, Relay.Direction.kForward);
			Shifter = new DoubleSolenoid(1, 2);
			Cylinder = new Solenoid(3);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize pneumatics due to a " + ExceptionInfo.getType(ex), ex);
		}
	}
}
