package org.usfirst.frc.team4342.api.logging;

import ernie.logging.data.ErrorLogData;
import ernie.logging.data.InfoLogData;
import ernie.logging.loggers.BaseLogger;
import ernie.logging.Severity;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Logger that logs to the console on the driver station
 */
public class RobotConsoleLogger extends BaseLogger 
{
	public static final String returnFeed = System.getProperty("line.separator");
	
	/**
	 * Logs to the console on the driver station. This method only
	 * logs the severity level and the message.
	 */
	@Override
	public void log(Severity severity, InfoLogData data) 
	{
		String mssg = "undefined";
		
		if(data instanceof ErrorLogData)
		{
			ErrorLogData eld = (ErrorLogData) data;
			mssg = createErrorMessage(severity, eld.getMessage(), eld.getException());
		}
		else
		{
			mssg = createMessage(severity, data.getMessage());
		}
		
		DriverStation.reportError(mssg, false);
	}
	
	/**
	 * Logs to the console on the driver station.
	 * @param severity the severity of the log
	 * @param message the message about the log
	 */
	public static void log(Severity severity, String message) 
	{
		String mssg = createMessage(severity, message);
		DriverStation.reportError(mssg, false);
	}
	
	private static String createMessage(Severity severity, String message) 
	{
		String mssg = severity.toString().toUpperCase() + ": " + message + returnFeed;
		
		return mssg;
	}
	
	private static String createErrorMessage(Severity severity, String message, Exception ex)
	{
		String mssg = severity.toString().toUpperCase() + ": " 
					  + message + "(" + ExceptionInfo.getType(ex) + ")" 
					  + returnFeed;
		
		return mssg;
	}
}