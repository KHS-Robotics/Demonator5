package org.usfirst.frc.team4342.api.logging;
import java.io.File;

import org.usfirst.frc.team4342.robot.components.Repository;

/**
 * This class is used to shift and get valid log files
 */
public class FileHelper 
{	
	private static final String ROOT = "/home/lvuser/";
	
	private FileHelper() {}
	
	/**
	 * Gets a valid log location and file
	 * @return a valid log file location
	 */
	public static File getValidLogFile() 
	{
		for(int i = 1; i <= 5; i++) 
		{
			File f = new File(ROOT + "Log[" + i + "].txt");
			
			if(!f.exists())
				return f;
		}
		
		shiftLogFiles();
		
		return new File(ROOT + "Log[1].txt");
	}
	
	/**
	 * Gets a valid log location and file
	 * @return a valid log file location
	 */
	public static File getValidPdpLogFile() 
	{
		for(int i = 1; i <= 5; i++) 
		{
			File f = new File(ROOT + "PdpLog[" + i + "].csv");
			
			if(!f.exists())
				return f;
		}
		
		shiftPdpLogFiles();
		
		return new File(ROOT + "PdpLog[1].csv");
	}
	
	public static File getValidNavXLogFile()
	{
		for(int i = 1; i <= 5; i++) 
		{
			File f = new File(ROOT + "NavXLog[" + i + "].csv");
			
			if(!f.exists())
				return f;
		}
		
		shiftNavXLogFiles();
		
		return new File(ROOT + "NavXLog[1].csv");
	}
	
	/**
	 * We can save up to 5 log files! Each time we make a new
	 * LocalLog, we want to check if we have to shift
	 * the log file index values up one and delete
	 * the oldest file and make way for the latest
	 * log file, [1].
	 */
	private static void shiftLogFiles() 
	{
		if(!new File(ROOT + "Log[5].txt").exists())
			return;
		
		new File(ROOT + "Log[5].txt").delete();
		
		for(int i = 4; i >= 1; i--) 
		{
			File f = new File(ROOT + "Log[" + i + "].txt");
			
			boolean renamed = f.renameTo(new File(ROOT + "Log[" + (i+1) + "].txt"));
			
			if(!renamed) 
			{
				Repository.ConsoleLog.warning("The file at path \"" + f.getPath() + "\" was not successfully renamed");
			}
		}
	}
	
	/**
	 * We can save up to 5 log files! Each time we make a new
	 * LocalLog, we want to check if we have to shift
	 * the log file index values up one and delete
	 * the oldest file and make way for the latest
	 * log file, [1].
	 */
	private static void shiftPdpLogFiles() 
	{	
		File lastFile = new File(ROOT + "PdpLog[5].csv");
		
		if(!lastFile.exists())
			return;
		
		lastFile.delete();
		
		for(int i = 4; i >= 1; i--) 
		{
			File f = new File(ROOT + "PdpLog[" + i + "].csv");
			
			boolean renamed = f.renameTo(new File(ROOT + "PdpLog[" + (i+1) + "].csv"));
			
			if(!renamed) 
			{
				Repository.ConsoleLog.warning("The file at path \"" + f.getPath() + "\" was not successfully renamed");
			}
		}
	}
	
	/**
	 * We can save up to 5 log files! Each time we make a new
	 * LocalLog, we want to check if we have to shift
	 * the log file index values up one and delete
	 * the oldest file and make way for the latest
	 * log file, [1].
	 */
	private static void shiftNavXLogFiles()
	{
		File lastFile = new File(ROOT + "NavXLog[5].csv");
		
		if(!lastFile.exists())
			return;
		
		lastFile.delete();
		
		for(int i = 4; i >= 1; i--) 
		{
			File f = new File(ROOT + "NavXLog[" + i + "].csv");
			
			boolean renamed = f.renameTo(new File(ROOT + "NavXLog[" + (i+1) + "].csv"));
			
			if(!renamed) 
			{
				Repository.ConsoleLog.warning("The file at path \"" + f.getPath() + "\" was not successfully renamed");
			}
		}
	}
}