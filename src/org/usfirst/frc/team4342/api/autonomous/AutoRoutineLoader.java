package org.usfirst.frc.team4342.api.autonomous;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import org.usfirst.frc.team4342.robot.components.Repository;

public final class AutoRoutineLoader 
{
	private AutoRoutineLoader() {}
	
	public static AutoRoutine getAutoRoutine(String path)
	{
		try
		{
			int id = loadFromTxtFile(path);
			
			switch(id)
			{
				
			}
			
			return AutoRoutine.Unknown;
		}
		catch(IOException ex)
		{
			Repository.Logs.error("Failed to load Auto Routine at " + path, ex);
			return AutoRoutine.Unknown;
		}
	}
	
	private static int loadFromTxtFile(String path) throws IOException
	{
		try
		{
			FileReader fr = new FileReader(path);
			BufferedReader br = new BufferedReader(fr);
			
			String number = br.readLine();
			br.close();
			
			return new Integer(number);
		}
		catch(NumberFormatException ex) 
		{
			return new Integer(9001);
		}
	}
}