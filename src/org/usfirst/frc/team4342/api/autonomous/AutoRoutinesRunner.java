package org.usfirst.frc.team4342.api.autonomous;

import org.usfirst.frc.team4342.api.drive.TankDrive;
import org.usfirst.frc.team4342.api.shooter.Shooter;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoRoutinesRunner 
{
	private AutoRoutinesRunner() {}
	
	private static boolean errored, finished;
	private static int currentStep, numLoops;
	
	// Current Step Enumeration: Start = 0, Defense = 1, Position = 2, Goal = 3, Finish = 4
	// RoutineStart: Nothing = 0, Load = 1, Grab = 2, Spy = 3
	// RoutineDefense: Nothing = 0, LowBar = 1, Moat = 2, Ramp = 3, RoughTerrain = 4, RockWall = 5
	// RoutinePosition: LowBar = 1, rest is 2-5
	// RoutineGoal: Nothing = 0, High = 1, Low = 2
	// RoutineFinish: Nothing(Stop) = 0, Neutral = 1, Secret = 2
	private static int RoutineStart, RoutineDefense, RoutinePosition, RoutineGoal, RoutineFinish;
	
	// Used for RoutineFinish
	private static boolean yawInitiallyOnTarget, fired, armInitiallyAtSetpoint;
	
	/**
	 * Executes the autonomous routine based on given auto data above
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
			
			putSmartDashobardData();
			
			if(currentStep == 0) // Routine Start
			{
				if(RoutineStart == 2 || RoutineStart == 3) // Grab ball or Spy
				{
					abort();
				}
				else if(RoutineStart == 1) // Start with ball
				{
					incrementStep();
				}
			}
			else if(currentStep == 1) // Routine Defense
			{
				if(RoutineDefense == 1) // Low Bar
				{
					if(Repository.TankDrive.autoLowBar(true, true, 0))
					{
						Repository.TankDrive.resetLowBarState();
						
						incrementStep();
					}
				}
				else if(RoutineDefense == 2) // Moat
				{
					if(Repository.TankDrive.autoMoat(true, true, 0))
					{
						Repository.TankDrive.resetMoatState();

						incrementStep();
					}
				}
				else if(RoutineDefense == 3) // Ramp Parts
				{
					if(Repository.TankDrive.autoRampParts(true, true, 0))
					{
						Repository.TankDrive.resetRampPartsState();
						
						incrementStep();
					}
				}
				else if(RoutineDefense == 4) // Rough Terrain
				{
					if(Repository.TankDrive.autoRoughTerrain(true, true, 0))
					{
						Repository.TankDrive.resetRoughTerrainState();
						
						incrementStep();
					}
				}
				else if(RoutineDefense == 5) // Rock Wall
				{
					if(Repository.TankDrive.autoRockWall(true, true, 0))
					{
						Repository.TankDrive.resetRockWallState();
						
						incrementStep();
					}
				}
			}
			else if(currentStep == 2) // Routine Position
			{
				if(RoutineDefense == 1 && RoutinePosition != 1) // Low Bar is always Position 1 and Defense 1. So why wouldn't they equal? Abort...
				{
					abort();
				}
				else if(RoutineDefense == 1) // From Low Bar
				{
					if(Repository.TankDrive.autoMoveDist(0.75, TankDrive.LOW_BAR_DIST_INCHES))
					{
						Repository.TankDrive.resetAutoMove();
						incrementStep();
					}
				}
				else if(RoutinePosition == 2) // From Position 2
				{
					if(Repository.TankDrive.autoMoveDist(0.75, TankDrive.SECOND_DEFENSE_DIST_INCHES))
					{
						Repository.TankDrive.resetAutoMove();
						incrementStep();
					}
				}
				else if(RoutinePosition == 3) // From Position 3, but we shoot from the courtyard so let's not move
				{
					incrementStep();
				}
				else if(RoutinePosition == 4) // From Position 4
				{
					if(Repository.TankDrive.autoMoveDist(0.75, TankDrive.FOURTH_DEFENSE_DIST_INCHES))
					{
						Repository.TankDrive.resetAutoMove();
						incrementStep();
					}
				}
				else if(RoutinePosition == 5) // From Position 5
				{
					if(Repository.TankDrive.autoMoveDist(0.75, TankDrive.FIFTH_DEFENSE_DIST_INCHES))
					{
						Repository.TankDrive.resetAutoMove();
						incrementStep();
					}
				}
			}
			else if(currentStep == 3) // Routine Goal
			{	
				if(RoutineGoal == 0) // Do nothing? Abort
				{
					abort();
				}
				else if(RoutineDefense == 1 || RoutinePosition == 2) // Low Bar or Position 2... Shooting angles are the same
				{
					Repository.TankDrive.goToAngle(TankDrive.BATTER_YAW);
					
					if(RoutineGoal == 1) // High Goal
					{
						Repository.Shooter.setArmSetpoint(Shooter.HIGH_BATTER_DIST);
					}
					else if(RoutineGoal == 2) // Low Goal
					{
						Repository.Shooter.setArmSetpoint(Shooter.LOW_GOAL_DIST);
					}
					
					if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint() && Repository.Shooter.shooterIsAtSetpoint())
					{
						Repository.Shooter.setBallPusher(true);
						fired = true;
					}
				}
				else if(RoutinePosition == 3) // From courtyard
				{
					Repository.TankDrive.goToAngle(TankDrive.THIRD_POSITION_YAW);
					
					if(RoutineGoal == 1) // High Goal
					{
						Repository.Shooter.setArmSetpoint(Shooter.THIRD_POSITION_HIGH_DIST);
					}
					else if(RoutineGoal == 2) // Low Goal
					{
						Repository.Shooter.setArmSetpoint(Shooter.THIRD_POSITION_LOW_DIST);
					}
					
					if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint() && Repository.Shooter.shooterIsAtSetpoint())
					{
						Repository.Shooter.setBallPusher(true);
						fired = true;
					}
				}
				else if(RoutinePosition == 4) // From Position 4
				{
					Repository.TankDrive.goToAngle(TankDrive.FOURTH_POSITION_YAW);
					
					if(RoutineGoal == 1) // High Goal
					{
						Repository.Shooter.setArmSetpoint(Shooter.FOURTH_POSITION_HIGH_DIST);
					}
					else if(RoutineGoal == 2) // Low Goal
					{
						Repository.Shooter.setArmSetpoint(Shooter.FOURTH_POSITION_LOW_DIST);
					}
					
					if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint() && Repository.Shooter.shooterIsAtSetpoint())
					{
						Repository.Shooter.setBallPusher(true);
						fired = true;
					}
				}
				else if(RoutinePosition == 5) // From Position 5
				{
					Repository.TankDrive.goToAngle(-TankDrive.BATTER_YAW); // Shooting from the right
					
					if(RoutineGoal == 1) // High Goal
					{
						Repository.Shooter.setArmSetpoint(Shooter.HIGH_BATTER_DIST);
					}
					else if(RoutineGoal == 2) // Low Goal
					{
						Repository.Shooter.setArmSetpoint(Shooter.LOW_GOAL_DIST);
					}
					
					if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint() && Repository.Shooter.shooterIsAtSetpoint())
					{
						Repository.Shooter.setBallPusher(true);
						fired = true;
					}
				}
				
				if(fired)
				{
					if(numLoops > 10)
					{
						Repository.Shooter.disableShooterPID();
						
						incrementStep();
					}
					
					numLoops++;
				}
				else if(RoutineGoal == 1) // High Goal, larger setpoint
				{
					Repository.Shooter.setShooterMotorsPID(85);
				}
				else if(RoutineGoal == 2) // Low Goal, smaller setpoint
				{
					Repository.Shooter.setShooterMotorsPID(60);
				}
			}
			else if(currentStep == 4) // Routine Finish
			{
				if(RoutineFinish == 0) // Do nothing? Abort
				{
					abort();
				}
				else if(RoutineFinish == 1) // Neutral zone
				{
					if(RoutinePosition == 3) // From courtyard
					{
						Repository.TankDrive.goToAngle(0);
						
						if(Repository.TankDrive.isAtAngleSetpoint() || yawInitiallyOnTarget)
						{
							yawInitiallyOnTarget = true;
							Repository.Shooter.setArmSetpoint(0);
							
							if(Repository.TankDrive.autoMoveDist(0.5, 6) && (Repository.Shooter.armIsAtSetpoint() || armInitiallyAtSetpoint))
							{
								armInitiallyAtSetpoint = true;
								
								if(RoutineDefense == 2) // Moat
								{
									if(Repository.TankDrive.autoMoat(false, true, 180))
									{
										incrementStep();
									}
								}
								else if(RoutineDefense == 3) // Ramp Parts
								{
									if(Repository.TankDrive.autoRampParts(false, true, 180))
									{
										incrementStep();
									}
								}
								else if(RoutineDefense == 4) // Rough Terrain
								{
									if(Repository.TankDrive.autoRoughTerrain(false, true, 180))
									{
										incrementStep();
									}
								}
								else if(RoutineDefense == 5) // Rock Wall
								{
									if(Repository.TankDrive.autoRockWall(false, true, 180))
									{
										incrementStep();
									}
								}
							}
						}
					}
					else // Not from courtyard, going to turn around toward drivers and go back over the defense
					{
						Repository.TankDrive.goToAngle(180);
						
						if(Repository.TankDrive.isAtAngleSetpoint() || yawInitiallyOnTarget)
						{
							yawInitiallyOnTarget = true;
							
							if(RoutineDefense == 1) // Low Bar
							{
								if(Repository.TankDrive.autoLowBar(true, true, 180))
								{
									incrementStep();
								}
							}
							else if(RoutineDefense == 2) // Moat
							{
								if(Repository.TankDrive.autoMoat(true, true, 180))
								{
									incrementStep();
								}
							}
							else if(RoutineDefense == 3) // Ramp Parts
							{
								if(Repository.TankDrive.autoRampParts(true, true, 180))
								{
									incrementStep();
								}
							}
							else if(RoutineDefense == 4) // Rough Terrain
							{
								if(Repository.TankDrive.autoRoughTerrain(true, true, 180))
								{
									incrementStep();
								}
							}
							else if(RoutineDefense == 5) // Rock Wall
							{
								if(Repository.TankDrive.autoRockWall(true, true, 180))
								{
									incrementStep();
								}
							}
						}
					}
				}
				else if(RoutineFinish == 2) // Secret Passage
				{
					abort();
				}
			}
			
			return finished;
		}
		catch(Exception ex)
		{
			if(!errored)
			{
				Repository.Logs.error("Unexpected error while executing auto", ex);
				abort();
				errored = true;
			}
			
			return false;
		}
	}
	
	public static void reset()
	{
		Repository.TankDrive.resetAutoDefense();
		Repository.TankDrive.resetAutoMove();
		yawInitiallyOnTarget = false;
		armInitiallyAtSetpoint = false;
		fired = false;
		finished = false;
		errored = false;
		currentStep = 0;
		numLoops = 0;
	}
	
	public static void abort()
	{
		Repository.TankDrive.stopAll();
		Repository.Shooter.stopAll();
		
		Repository.Logs.warning(
			"Aborted autonomous :: "
			+ "RS=" + RoutineStart + ", "
			+ "RD=" + RoutineDefense + ", "
			+ "RP=" + RoutinePosition + ", "
			+ "RG=" + RoutineGoal + ", "
			+ "RF=" + RoutineFinish +", "
			+ "CS=" + currentStep + ", "
			+ "YIOT=" + yawInitiallyOnTarget +", "
			+ "AIOT=" + armInitiallyAtSetpoint + ", "
			+ "LBS" + Repository.TankDrive.getLowBarState()
			+ "MS" + Repository.TankDrive.getMoatState()
			+ "RPS" + Repository.TankDrive.getRampPartsState()
			+ "RTS" + Repository.TankDrive.getRoughTerrainState()
			+ "RWS" + Repository.TankDrive.getRockWallState()
			+ "F=" + fired + ", "
			+ "NL=" + numLoops + ", "
			+ "E=" + errored + ", "
			+ "Time=" + Repository.Timer.get() + "s"
		);
		
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
}