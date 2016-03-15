package org.usfirst.frc.team4342.api.autonomous;

import static org.usfirst.frc.team4342.robot.components.Repository.Logs;
import static org.usfirst.frc.team4342.robot.components.Repository.Shooter;
import static org.usfirst.frc.team4342.robot.components.Repository.TankDrive;

public class AutoRoutinesRunner 
{
	private AutoRoutinesRunner() {}
	
	private static boolean errored, finished;
	private static AutoRoutine lastRoutine;
	private static int currentStep, numLoops;
	
	/**
	 * 
	 * @param routine the routine to execute
	 * @return true if the routine is finished; false otherwise
	 */
	public static boolean execute(AutoRoutine routine)
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
			
			return finished;
		}
		catch(Exception ex)
		{
			if(!errored)
			{
				Logs.error("Unexpected error while executing " + routine.toString() + " auto routine", ex);
				errored = true;
			}
			
			return false;
		}
	}
	
	public static void reset()
	{
		finished = false;
		errored = false;
		currentStep = 0;
		numLoops = 0;
	}
	
	public static AutoRoutine getLastAutoRoutine()
	{
		return lastRoutine;
	}
	
	public static int getCurrentStep()
	{
		return currentStep;
	}
	
	public static boolean hasErrored()
	{
		return errored;
	}
	
	public static boolean isFinished()
	{
		return finished;
	}
	
	private static void executeRampPartsRoutine()
	{
		if(currentStep == 0)
		{
			if(TankDrive.autoRampParts(true, true, 0))
			{
				currentStep++;
				finished = true;
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
				finished = true;
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
				finished = true;
			}
		}
	}
	
	private static void executeLowBarRoutine()
	{
		if(currentStep == 0)
		{
			Shooter.setArmSetpoint(Shooter.LOW_BAR_ENC_DIST);
			
			if(TankDrive.autoLowBar(true, true, Shooter.armIsAtSetpoint(), 0))
			{
				currentStep++;
				finished = true;
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
				finished = true;
			}
		}
	}
	
	private static void executeLowBarAndShootBatterRoutine()
	{
		if(currentStep == 0)
		{
			Shooter.setArmSetpoint(Shooter.LOW_BAR_ENC_DIST);
			
			if(TankDrive.autoLowBar(true, Shooter.armIsAtSetpoint(), true, 0))
			{
				Shooter.setArmSetpoint(Shooter.LOW_BAR_SHOOT_ANGLE);
				currentStep++;
			}
		}
		else if(currentStep == 1)
		{
			if(TankDrive.autoMoveDist(0.75, TankDrive.LOW_BAR_DIST_INCHES))
			{
				TankDrive.goToAngle(TankDrive.LOW_BAR_YAW);
				Shooter.setShooterMotorsPID(85);
				currentStep++;
			}
		}
		else if(currentStep == 2)
		{
			if(TankDrive.isAtAngleSetpoint() && Shooter.armIsAtSetpoint() && Shooter.shooterIsAtSetpoint())
			{
				Shooter.setBallPusher(true);
				currentStep++;
			}
		}
		else if(currentStep == 3)
		{
			if(numLoops >= 10)
			{
				TankDrive.goToAngle(180);
				Shooter.setArmSetpoint(Shooter.LOW_BAR_ENC_DIST);
				Shooter.setShooterMotorsPID(0);
				Shooter.setBallPusher(false);
				
				currentStep++;
			}
			
			numLoops++;
		}
		else if(currentStep == 4)
		{
			if(TankDrive.isAtAngleSetpoint() && Shooter.armIsAtSetpoint())
			{
				TankDrive.autoMoveDist(0.75, TankDrive.LOW_BAR_DIST_INCHES);
				TankDrive.resetLowBarState();
				currentStep++;
			}
		}
		else if(currentStep == 5)
		{
			if(TankDrive.autoLowBar(true, true, false, 0))
			{
				currentStep++;
				finished = true;
			}
		}
	}
}