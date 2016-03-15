package org.usfirst.frc.team4342.api.autonomous;

import static org.usfirst.frc.team4342.robot.components.Repository.Logs;
import static org.usfirst.frc.team4342.robot.components.Repository.Shooter;
import static org.usfirst.frc.team4342.robot.components.Repository.TankDrive;
import static org.usfirst.frc.team4342.robot.components.Repository.DriveTrain;

public class AutoRoutinesRunner 
{
	private AutoRoutinesRunner() {}
	
	private static double yawOffset;
	
	private static boolean errored;
	private static AutoRoutine lastRoutine;
	private static int currentStep, numLoops;
	
	public static void execute(AutoRoutine routine)
	{
		try
		{
			lastRoutine = routine;
			
			switch(routine)
			{
				case RampParts:
					break;
				case RoughTerrain:
					break;
				case Moat:
					break;
				case LowBar:
					break;
				case RockWall:
					break;
				default:
					if(!errored)
						Logs.warning("Unknown auto routine selected (Value=" + routine.getId());
					errored = true;
					break;
			}
		}
		catch(Exception ex)
		{
			if(!errored)
			{
				Logs.error("Unexpected error while executing " + routine.toString() + " auto routine", ex);
				errored = true;
			}
		}
	}
	
	public static void reset()
	{
		errored = false;
		currentStep = 0;
	}
	
	public static AutoRoutine getLastAutoRoutine()
	{
		return lastRoutine;
	}
	
	public static int getCurrentStep()
	{
		return currentStep;
	}
	
	public static double getYawOffset()
	{
		return yawOffset;
	}
	
	public static void executeRampPartsRoutine()
	{
		if(currentStep == 0)
		{
			
		}
	}
}