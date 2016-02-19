package org.usfirst.frc.team4342.api.logging;

import java.io.BufferedWriter;
import java.io.FileWriter;

import org.usfirst.frc.team4342.api.drive.DriveTrain;
import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

public class NavXLogger 
{
	private NavXLogger() {}
	
	private static boolean isLogging;
	
	public static void startLogging(AHRS navx, DriveTrain drive)
	{
		if(isLogging)
			return;
		
		new NavXLoggingThread(navx, drive).start();
		isLogging = true;
	}
	
	public static void stopLogging()
	{
		isLogging = false;
	}
	
	private static class NavXLoggingThread extends Thread implements Runnable
	{
		private AHRS navx;
		private DriveTrain drive;
		private static BufferedWriter writer;
		
		public NavXLoggingThread(AHRS navx, DriveTrain drive)
		{
			this.navx = navx;
			this.drive = drive;
		}
		
		@Override
		public void run()
		{
			init();
			
			while(isLogging)
			{
				try
				{
					
				}
				catch(Exception ex)
				{
					Repository.Log.error("Failed to log NavX data!", ex);
					Repository.ConsoleLog.warning("Failed to log NavX data!");
					isLogging = false;
				}
			}
		}
		
		private void init()
		{
			try
			{
				writer = new BufferedWriter(new FileWriter(FileHelper.getValidNavXLogFile()));
			}
			catch(Exception ex)
			{
				Repository.Log.error("Failed to initialize NavX logger!", ex);
				Repository.ConsoleLog.warning("Failed to log NavX data!");
				isLogging = false;
			}
		}
	}
}
