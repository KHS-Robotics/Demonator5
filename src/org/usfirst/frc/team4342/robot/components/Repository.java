package org.usfirst.frc.team4342.robot.components;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.api.logging.PdpLogger;
import org.usfirst.frc.team4342.api.logging.RobotConsoleLogger;
import org.usfirst.frc.team4342.api.logging.RobotLogFactory;
import org.usfirst.frc.team4342.api.pdp.PdpInfoExtractor;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
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
	
	public static CANTalon FrontRight, FrontLeft, MiddleRight, MiddleLeft, 
						   RearRight, RearLeft;
	
	public static Joystick DriveStick;
	
	public static AHRS Navx;
	
	public static void initializeAll()
	{
		if(!initialized)
		{
			initializeLogs();
			initializeJoysticks();
			initializeDrive();
		}
		
		initialized = true;
	}
	
	private static void initializeLogs()
	{
		try
		{
			ConsoleLog = RobotLogFactory.createRobotConsoleLogger();
			Log = RobotLogFactory.createLocalLog();
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
			MiddleRight = new CANTalon(1);
			RearRight = new CANTalon(2);
			
			FrontLeft = new CANTalon(15);
			MiddleLeft = new CANTalon(14);
			RearLeft = new CANTalon(13);
			
			FrontRight.configEncoderCodesPerRev(512);
			FrontRight.enableBrakeMode(true);
			
			FrontLeft.configEncoderCodesPerRev(512);
			FrontLeft.enableBrakeMode(true);
			
			MiddleRight.configEncoderCodesPerRev(512);
			MiddleRight.enableBrakeMode(true);
			
			MiddleLeft.configEncoderCodesPerRev(512);
			MiddleLeft.enableBrakeMode(true);
			
			RearRight.configEncoderCodesPerRev(512);
			RearRight.enableBrakeMode(true);
			
			RearLeft.configEncoderCodesPerRev(512);
			RearLeft.enableBrakeMode(true);
			
			FrontRight.enable();
			FrontLeft.enable();
			MiddleRight.enable();
			MiddleLeft.enable();
			RearRight.enable();
			RearLeft.enable();
			
			Navx = new AHRS(SPI.Port.kMXP, (byte)50);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize drive due to a " + ExceptionInfo.getType(ex), ex);
		}
	}
}
