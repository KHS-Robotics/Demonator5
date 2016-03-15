package org.usfirst.frc.team4342.api.autonomous;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class AutoRoutineLoader 
{
	private AutoRoutineLoader() {}
	
	public static AutoRoutine getAutoRoutine(int id)
	{
		switch(id)
		{
			case 0:
				return AutoRoutine.RampParts;
			case 1:
				return AutoRoutine.RoughTerrain;
			case 2:
				return AutoRoutine.Moat;
			case 3:
				return AutoRoutine.LowBar;
			case 4:
				return AutoRoutine.LowBarAndShoot;
			case 5:
				return AutoRoutine.RockWall;
		}
		
		return AutoRoutine.Unknown;
	}
	
	public static AutoRoutine getAutoRoutine(String path)
	{
		try
		{
			int id = loadFromTxtFile(path);
			SmartDashboard.putNumber("Auto-Routine", id);
			
			switch(id)
			{
				case 0:
					return AutoRoutine.RampParts;
				case 1:
					return AutoRoutine.RoughTerrain;
				case 2:
					return AutoRoutine.Moat;
				case 3:
					return AutoRoutine.LowBar;
				case 4:
					return AutoRoutine.RockWall;
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
