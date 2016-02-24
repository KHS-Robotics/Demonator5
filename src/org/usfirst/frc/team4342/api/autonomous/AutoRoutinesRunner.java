package org.usfirst.frc.team4342.api.autonomous;

import org.usfirst.frc.team4342.robot.components.Repository;

public class AutoRoutinesRunner 
{
	private AutoRoutinesRunner() {}
	
	private static boolean logged;
	private static AutoRoutine lastRoutine;
	private static int currentStep;
	
	public static void execute(AutoRoutine routine)
	{
		lastRoutine = routine;
		
		switch(routine)
		{
			case RampParts:
				Repository.TankDrive.autoRampParts(true, 0.75, true, 0);
				break;
			case RoughTerrain:
				Repository.TankDrive.autoRoughTerrain(true, 0, true, 0);
				break;
			case Moat:
				break;
			case LowBar:
				break;
			case RockWall:
				break;
			default:
				if(!logged)
					Repository.Logs.warning("Unknown auto routine selected (Value=" + routine.getId());
				logged = true;
				break;
		}
	}
	
	public static void reset()
	{
		logged = false;
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
}