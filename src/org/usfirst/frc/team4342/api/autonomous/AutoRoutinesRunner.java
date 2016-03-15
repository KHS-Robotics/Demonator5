package org.usfirst.frc.team4342.api.autonomous;

import static org.usfirst.frc.team4342.robot.components.Repository.Logs;
import static org.usfirst.frc.team4342.robot.components.Repository.Shooter;
import static org.usfirst.frc.team4342.robot.components.Repository.TankDrive;

public class AutoRoutinesRunner 
{
	private AutoRoutinesRunner() {}
	
	private static boolean errored;
	private static AutoRoutine lastRoutine;
	private static int currentStep;
	
	public static void execute(AutoRoutine routine)
	{
		try
		{
			lastRoutine = routine;
			
			switch(routine)
			{
				case RampParts:
					executeRampPartsRoutine();
				break;
				case RoughTerrain:
					executeRoughTerrainRoutine();
				break;
				case Moat:
					executeMoatRoutine();
				break;
				case LowBar:
					executeLowBarRoutine();
				break;
				case RockWall:
					executeRockWallRoutine();
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
	
	private static void executeRampPartsRoutine()
	{
		if(currentStep == 0)
		{
			if(TankDrive.autoRampParts(false, false, true, 0, 180.0))
			{
				currentStep++;
			}
		}
	}
	
	private static void executeRoughTerrainRoutine()
	{
		if(currentStep == 0)
		{
			if(TankDrive.autoRoughTerrain(false, true, true, 0, 0))
			{
				currentStep++;
			}
		}
	}
	
	private static void executeMoatRoutine()
	{
		if(currentStep == 0)
		{
			if(TankDrive.autoMoat(false, true, true, 0, 0))
			{
				currentStep++;
			}
		}
	}
	
	private static void executeLowBarRoutine()
	{
		if(currentStep == 0)
		{
			Shooter.setArmSetpoint(400);
			
			if(TankDrive.autoLowBar(false, true, true, Shooter.armIsAtSetpoint(), 0, 0))
			{
				currentStep++;
			}
		}
	}
	
	private static void executeRockWallRoutine()
	{
		if(currentStep == 0)
		{
			if(TankDrive.autoRockWall(false, true, true, 0, 0))
			{
				currentStep++;
			}
		}
	}
}