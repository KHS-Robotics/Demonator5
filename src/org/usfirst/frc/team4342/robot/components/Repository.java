package org.usfirst.frc.team4342.robot.components;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.api.logging.PdpLogger;
import org.usfirst.frc.team4342.api.logging.RobotConsoleLogger;
import org.usfirst.frc.team4342.api.logging.RobotLogFactory;
import org.usfirst.frc.team4342.api.pdp.PdpInfoExtractor;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
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
	public static MultiLogger logs;
	
	public static PdpInfoExtractor Pdp;
	
	public static CANTalon frontRight, frontLeft, rearRight, rearLeft;
	
	public static Joystick DriveStick;
	
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
			logs = new MultiLogger(new ILogger[] { ConsoleLog, Log });
		}
		catch(Exception ex)
		{
			ConsoleLog.warning("Failed to initialize log due to a " + ExceptionInfo.getType(ex));
			logs = new MultiLogger(new ILogger[] { ConsoleLog });
		}
		
		try
		{
			Pdp = new PdpInfoExtractor();
			PdpLogger.start(Pdp, Log, ConsoleLog);
		}
		catch(Exception ex)
		{
			logs.warning("Failed to start PDP Logger due to a " + ExceptionInfo.getType(ex));
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
			logs.error("Failed to initialize joysticks due to a " + ExceptionInfo.getType(ex), ex);
		}
	}
	
	private static void initializeDrive()
	{
		try
		{
			frontRight = new CANTalon(0);
			frontLeft = new CANTalon(1);
			rearRight = new CANTalon(2);
			rearLeft = new CANTalon(3);
			
			frontRight.configEncoderCodesPerRev(512);
			frontRight.enableBrakeMode(true);
			
			frontLeft.configEncoderCodesPerRev(512);
			frontLeft.enableBrakeMode(true);
			
			rearRight.configEncoderCodesPerRev(512);
			rearRight.enableBrakeMode(true);
			
			rearLeft.configEncoderCodesPerRev(512);
			rearRight.enableBrakeMode(true);
			
			frontRight.enable();
			frontLeft.enable();
			rearRight.enable();
			rearLeft.enable();
		}
		catch(Exception ex)
		{
			logs.error("Failed to initialize drive due to a " + ExceptionInfo.getType(ex), ex);
		}
	}
}
