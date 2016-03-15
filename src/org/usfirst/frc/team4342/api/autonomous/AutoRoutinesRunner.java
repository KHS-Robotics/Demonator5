package org.usfirst.frc.team4342.api.autonomous;

import static org.usfirst.frc.team4342.robot.components.Repository.Logs;
import static org.usfirst.frc.team4342.robot.components.Repository.Shooter;
import static org.usfirst.frc.team4342.robot.components.Repository.TankDrive;

public class AutoRoutinesRunner 
{
	private AutoRoutinesRunner() {}
	
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
				case LowBarAndShoot:
					executeLowBarAndShootRoutine();
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
			if(TankDrive.autoRampParts(true, true, 0))
			{
				currentStep++;
			}
		}
	}
	
	private static void executeRoughTerrainRoutine()
	{
		if(currentStep == 0)
		{
			if(TankDrive.autoRoughTerrain(true, true, 0))
			{
				currentStep++;
			}
		}
	}
	
	private static void executeMoatRoutine()
	{
		if(currentStep == 0)
		{
			if(TankDrive.autoMoat(true, true, 0))
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
			
			if(TankDrive.autoLowBar(true, true, Shooter.armIsAtSetpoint(), 0))
			{
				currentStep++;
			}
		}
	}
	
	private static void executeLowBarAndShootRoutine()
	{
		if(currentStep == 0)
		{
			Shooter.setArmSetpoint(400);
			
			if(TankDrive.autoLowBar(true, true, Shooter.armIsAtSetpoint(), 0))
			{
				currentStep++;
			}
		}
		else if(currentStep == 1)
		{
			Shooter.setShooterMotorsPID(85);
			Shooter.setArmSetpoint(300);
			
			if(Shooter.shooterIsAtSetpoint() && Shooter.armIsAtSetpoint())
			{
				Shooter.setBallPusher(true);
				numLoops++;

				if(numLoops > 10)
				{
					currentStep++;
				}
			}
		}
	}
	
	private static void executeRockWallRoutine()
	{
		if(currentStep == 0)
		{
			if(TankDrive.autoRockWall(true, true, 0))
			{
				currentStep++;
			}
		}
	}
}