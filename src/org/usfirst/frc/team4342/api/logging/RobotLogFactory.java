package org.usfirst.frc.team4342.api.logging;
import java.io.IOException;

import org.usfirst.frc.team4342.api.logging.FileHelper;
import org.usfirst.frc.team4342.api.logging.RobotConsoleLogger;

import ernie.logging.loggers.LocalLogger;
import ernie.logging.loggers.LoggerAsync;

/**
 * Factory to create loggers for the robot
 */
public class RobotLogFactory 
{
	public static final String ROOT = "/home/lvuser/";
	
	/**
	 * Creates a simple logger that logs to a specified text file.
	 * WARNING: LocalLog is not thread safe, consider using
	 * LoggingThread for a thread safe logger
	 * @param clearLogs true to clear the log file, false to append
	 * @return a logger that logs to a text file
	 * @throws IOException
	 * @throws LoggingException 
	 */
	public static LocalLogger createLocalLog() throws IOException 
	{
		return new LocalLogger("Demonator5", FileHelper.getValidLogFile(), true);
	}
	
	/**
	 * Creates a thread safe logger that logs in the background of
	 * the program to a specified text file
	 * @param clearLogs true to clear the log file, false to append
	 * @return a logger that logs to a text file
	 * @throws IOException
	 * @throws LoggingException 
	 */
	public static LoggerAsync createAsyncLogger() throws IOException 
	{
		return new LoggerAsync(createLocalLog());
	}
	
	/**
	 * Creates a new logger for the console on the Driver Station
	 * @return a logger to log to the console on the DS
	 */
	public static RobotConsoleLogger createRobotConsoleLogger() 
	{
		return new RobotConsoleLogger();
	}
}
