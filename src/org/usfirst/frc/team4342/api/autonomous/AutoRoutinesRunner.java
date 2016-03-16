package org.usfirst.frc.team4342.api.autonomous;

import org.usfirst.frc.team4342.api.drive.TankDrive;
import org.usfirst.frc.team4342.api.shooter.Shooter;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoRoutinesRunner 
{
	private AutoRoutinesRunner() {}
	
	private static boolean errored, finished;
	private static AutoRoutine lastRoutine;
	private static int currentStep, numLoops;
	
	// Current Step Enumeration: Start = 0, Defense = 1, Position = 2, Goal = 3, Finish = 4
	// RoutineStart: Nothing = 0, Load = 1, Grab = 2, Spy = 3
	// RoutineDefense: Nothing = 0, LowBar = 1, Moat = 2, Ramp = 3, RoughTerrain = 4, RockWall = 5
	// RoutinePosition: LowBar = 1, rest is 2-5
	// RoutineGoal: Nothing = 0, High = 1, Low = 2
	// RoutineFinish: Nothing(Stop) = 0, Neutral = 1, Secret = 2
	private static int RoutineStart, RoutineDefense, RoutinePosition, RoutineGoal, RoutineFinish;
	
	/**
	 * 
	 * @param routine the routine to execute
	 * @return true if the routine is finished; false otherwise
	 */
	public static boolean execute()
	{
		try
		{
			if(Repository.Timer.hasPeriodPassed(3) || RoutineStart == 0 || RoutineDefense == 0 || RoutinePosition == 0)
			{
				abort();
				
				return true;
			}
			
			if(currentStep == 0)
			{
				if(RoutineStart == 2 || RoutineStart == 3)
				{
					abort();
				}
				else if(RoutineStart == 1)
				{
					incrementStep();
				}
			}
			else if(currentStep == 1)
			{
				if(RoutineDefense == 1)
				{
					if(Repository.TankDrive.autoLowBar(true, true, 0))
					{
						Repository.TankDrive.resetLowBarState();
						
						incrementStep();
					}
				}
				else if(RoutineDefense == 2)
				{
					if(Repository.TankDrive.autoMoat(true, true, 0))
					{
						Repository.TankDrive.resetMoatState();

						incrementStep();
					}
				}
				else if(RoutineDefense == 3)
				{
					if(Repository.TankDrive.autoRampParts(true, true, 0))
					{
						Repository.TankDrive.resetRampPartsState();
						
						incrementStep();
					}
				}
				else if(RoutineDefense == 4)
				{
					if(Repository.TankDrive.autoRoughTerrain(true, true, 0))
					{
						Repository.TankDrive.resetRoughTerrainState();
						
						incrementStep();
					}
				}
				else if(RoutineDefense == 5)
				{
					if(Repository.TankDrive.autoRockWall(true, true, 0))
					{
						Repository.TankDrive.resetRockWallState();
						
						incrementStep();
					}
				}
			}
			else if(currentStep == 2)
			{
				if(RoutineDefense == 1 && RoutinePosition != 1)
				{
					abort();
				}
				else if(RoutineDefense == 1)
				{
					if(Repository.TankDrive.autoMoveDist(0.75, TankDrive.LOW_BAR_DIST_INCHES))
					{
						incrementStep();
					}
				}
				else if(RoutinePosition == 2)
				{
					if(Repository.TankDrive.autoMoveDist(0.75, TankDrive.SECOND_DEFENSE_DIST_INCHES))
					{
						incrementStep();
					}
				}
				else if(RoutinePosition == 3)
				{
					incrementStep();
				}
				else if(RoutinePosition == 4)
				{
					if(Repository.TankDrive.autoMoveDist(0.75, TankDrive.FOURTH_DEFENSE_DIST_INCHES))
					{
						incrementStep();
					}
				}
				else if(RoutinePosition == 5)
				{
					if(Repository.TankDrive.autoMoveDist(0.75, TankDrive.FIFTH_DEFENSE_DIST_INCHES))
					{
						incrementStep();
					}
				}
			}
			else if(currentStep == 3)
			{	
				if(RoutineGoal == 0)
				{
					abort();
				}
				else if(RoutineDefense == 1 || RoutinePosition == 2)
				{
					Repository.TankDrive.goToAngle(TankDrive.BATTER_YAW);
					
					if(RoutineGoal == 1)
					{
						Repository.Shooter.setArmSetpoint(Shooter.HIGH_BATTER_ANGLE);
					}
					else if(RoutineGoal == 2)
					{
						Repository.Shooter.setArmSetpoint(Shooter.LOW_BATTER_ANGLE);
					}
					
					if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint() && Repository.Shooter.shooterIsAtSetpoint())
					{
						Repository.Shooter.setBallPusher(true);
						
						incrementStep();
					}
				}
				else if(RoutinePosition == 3)
				{
					Repository.TankDrive.goToAngle(TankDrive.THIRD_POSITION_YAW);
					
					if(RoutineGoal == 1)
					{
						Repository.Shooter.setArmSetpoint(Shooter.THIRD_POSITION_HIGH_ANGLE);
					}
					else if(RoutineGoal == 2)
					{
						Repository.Shooter.setArmSetpoint(Shooter.THIRD_POSITION_LOW_DIST);
					}
					
					if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint() && Repository.Shooter.shooterIsAtSetpoint())
					{
						Repository.Shooter.setBallPusher(true);
						
						incrementStep();
					}
				}
				else if(RoutinePosition == 4)
				{
					Repository.TankDrive.goToAngle(TankDrive.FOURTH_POSITION_YAW);
					
					if(RoutineGoal == 1)
					{
						Repository.Shooter.setArmSetpoint(Shooter.FOURTH_POSITION_HIGH_DIST);
					}
					else if(RoutineGoal == 2)
					{
						Repository.Shooter.setArmSetpoint(Shooter.FOURTH_POSITION_LOW_DIST);
					}
					
					if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint() && Repository.Shooter.shooterIsAtSetpoint())
					{
						Repository.Shooter.setBallPusher(true);
						
						incrementStep();
					}
				}
				else if(RoutinePosition == 5)
				{
					Repository.TankDrive.goToAngle(-TankDrive.BATTER_YAW);
					
					if(RoutineGoal == 1)
					{
						Repository.Shooter.setArmSetpoint(Shooter.HIGH_BATTER_ANGLE);
					}
					else if(RoutineGoal == 2)
					{
						Repository.Shooter.setArmSetpoint(Shooter.LOW_BATTER_ANGLE);
					}
					
					if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint() && Repository.Shooter.shooterIsAtSetpoint())
					{
						Repository.Shooter.setBallPusher(true);
						
						incrementStep();
					}
				}
				
				if(RoutineGoal == 1)
				{
					Repository.Shooter.setShooterMotorsPID(85);
				}
				else if(RoutineGoal == 2)
				{
					Repository.Shooter.setShooterMotorsPID(60);
				}
			}
			else if(currentStep == 4)
			{
				if(RoutineFinish == 0)
				{
					abort();
				}
				else if(RoutineFinish == 1)
				{
					
				}
			}
			
			putSmartDashobardData();
			
			return finished;
		}
		catch(Exception ex)
		{
			if(!errored)
			{
				Repository.Logs.error("Unexpected error while executing auto", ex);
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
	
	public static void abort()
	{
		//Repository.Repository.TankDrive.stopAll();
		//Repository.Repository.Shooter.stopAll();
		
		currentStep = 9001;
		finished = true;
		
		Repository.Timer.stop();
	}

	public static void incrementStep()
	{
		Repository.Timer.reset();
		currentStep++;
	}
	
	public static void setRoutineData(int start, int defense, int position, int goal, int finish)
	{
		RoutineStart = start;
		RoutineDefense = defense;
		RoutinePosition = position;
		RoutineGoal = goal;
		RoutineFinish = finish;
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
	
	private static void putSmartDashobardData()
	{
		SmartDashboard.putNumber("AutoStep", currentStep);
		SmartDashboard.putBoolean("AutoFinished", finished);
		SmartDashboard.putBoolean("AutoErrored", errored);
		SmartDashboard.putNumber("AutoTimerSeconds", Repository.Timer.get());
	}
	
	private static void executeRampPartsRoutine()
	{
		if(currentStep == 0)
		{
			if(Repository.TankDrive.autoRampParts(true, true, 0))
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
			if(Repository.TankDrive.autoRoughTerrain(true, true, 0))
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
			if(Repository.TankDrive.autoMoat(true, true, 0))
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
			if(Repository.TankDrive.autoLowBar(true, true, 0))
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
			if(Repository.TankDrive.autoRockWall(true, true, 0))
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
			if(Repository.TankDrive.autoLowBar(true, true, 0))
			{
				Repository.Shooter.setArmSetpoint(Shooter.LOW_BAR_SHOOT_ANGLE);
				currentStep++;
			}
		}
		else if(currentStep == 1)
		{
			if(Repository.TankDrive.autoMoveDist(0.75, TankDrive.LOW_BAR_DIST_INCHES))
			{
				Repository.TankDrive.goToAngle(TankDrive.LOW_BAR_YAW);
				Repository.Shooter.setShooterMotorsPID(85);
				currentStep++;
			}
		}
		else if(currentStep == 2)
		{
			if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint() && Repository.Shooter.shooterIsAtSetpoint())
			{
				Repository.Shooter.setBallPusher(true);
				currentStep++;
			}
		}
		else if(currentStep == 3)
		{
			if(numLoops >= 10)
			{
				Repository.TankDrive.goToAngle(180);
				
				Repository.Shooter.setShooterMotorsPID(0);
				Repository.Shooter.setBallPusher(false);
				
				currentStep++;
			}
			
			numLoops++;
		}
		else if(currentStep == 4)
		{
			if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint())
			{
				Repository.TankDrive.autoMoveDist(0.75, TankDrive.LOW_BAR_DIST_INCHES);
				Repository.TankDrive.resetLowBarState();
				currentStep++;
			}
		}
		else if(currentStep == 5)
		{
			if(Repository.TankDrive.autoLowBar(true, false, 0))
			{
				currentStep++;
				finished = true;
			}
		}
	}
}