package org.usfirst.frc.team4342.api.autonomous;

public class AutoRoutinesRunner 
{
	private AutoRoutinesRunner() {}
	
	private static int currentStep;
	
	public static void execute(AutoRoutine routine)
	{
		switch(routine)
		{
			
		}
	}
	
	public static void reset()
	{
		currentStep = 0;
	}
	
	public static int getCurrentStep()
	{
		return currentStep;
	}
}
