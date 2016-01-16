package org.usfirst.frc.team4342.robot.components;

import org.usfirst.frc.team4342.robot.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.robot.api.logging.PdpLogger;
import org.usfirst.frc.team4342.robot.api.logging.RobotConsoleLogger;
import org.usfirst.frc.team4342.robot.api.logging.RobotLogFactory;
import org.usfirst.frc.team4342.robot.api.pdp.PdpInfoExtractor;

import edu.wpi.first.wpilibj.Joystick;
import ernie.logging.loggers.ILogger;
import ernie.logging.loggers.MultiLogger;

public class Repository 
{
	private static boolean initialized;
	
	protected static ILogger Log;
	protected static RobotConsoleLogger ConsoleLog;
	protected static MultiLogger logs;
	
	protected static PdpInfoExtractor Pdp;
	
	protected static Joystick DriveStick;
	
	public static void initializeAll()
	{
		if(!initialized)
		{
			initializeLogs();
			initializeJoysticks();
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
			logs.error("Failed to initialize joysticks", ex);
		}
	}
}
