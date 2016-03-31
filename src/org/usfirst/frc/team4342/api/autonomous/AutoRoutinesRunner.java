package org.usfirst.frc.team4342.api.autonomous;

import org.usfirst.frc.team4342.api.drive.TankDrive;
import org.usfirst.frc.team4342.api.shooter.Shooter;
import org.usfirst.frc.team4342.robot.Repository;

public class AutoRoutinesRunner 
{
	private AutoRoutinesRunner() {}
	
	private static boolean errored, finished;
	private static int currentStep, numLoops;
	
	// Current Step Enumeration: Start = 0, Defense = 1, Position = 2, Goal = 3, Finish = 4
	// RoutineStart: Nothing = 0, Load = 1, Grab = 2, Spy = 3
	// RoutineDefense: Nothing = 0, LowBar = 1, Moat = 2, Ramp = 3, RoughTerrain = 4, RockWall = 5
	// RoutinePosition: LowBar = 1, rest is 2-5
	// RoutineGoal: Nothing = 0, High = 1, Low = 2, Spit = 3
	// RoutineFinish: Nothing(Stop) = 0, Neutral = 1, Secret = 2
	private static int RoutineStart, RoutineDefense, RoutinePosition, RoutineGoal, RoutineFinish;
	private static RoutineData data;
	
	// Used for RoutineFinish
	private static boolean yawInitiallyOnTarget, fired, armInitiallyAtSetpoint;
	
	private static int timeoutSeconds = 5; // 5 by default
	
	/**
	 * Executes the autonomous routine based on given auto data above
	 * @return true if the routine is finished; false otherwise
	 */
	public static boolean execute()
	{
		try
		{
			if(Repository.Timer.hasPeriodPassed(timeoutSeconds))
				abort("Timer has expired");
			if(RoutineStart <= 0)
				abort("RoutineStart cannot be less than or equal 0");
			if(RoutineDefense <= 0)
				abort("RoutineDefense cannot be less than or equal 0");
			if(RoutinePosition <= 0)
				abort("RoutinePosition cannot be less than or equal 0");
			
			if(currentStep == 0) // Routine Start
			{
				if(RoutineStart == 2 || RoutineStart == 3) // Grab ball or Spy
				{
					abort("RoutineStart cannot equal 2 or 3. These numbers have yet to be implemented!");
				}
				else if(RoutineStart == 1) // Start with ball
				{
					incrementStep("Starting with a ball");
				}
			}
			else if(currentStep == 1) // Routine Defense
			{
				if(RoutineDefense == 1) // Low Bar
				{
					if(Repository.TankDrive.autoLowBar(true, true, 0))
					{
						Repository.TankDrive.resetLowBarState();
						
						incrementStep("Done going thru low bar");
					}
				}
				else if(RoutineDefense == 2) // Moat
				{
					if(Repository.TankDrive.autoMoat(true, true, 0))
					{
						Repository.TankDrive.resetMoatState();

						incrementStep("Done going over moat");
					}
				}
				else if(RoutineDefense == 3) // Ramp Parts
				{
					if(Repository.TankDrive.autoRampParts(true, true, 0))
					{
						Repository.TankDrive.resetRampPartsState();
						
						incrementStep("Done going over ramp parts");
					}
				}
				else if(RoutineDefense == 4) // Rough Terrain
				{
					if(Repository.TankDrive.autoRoughTerrain(true, true, 0))
					{
						Repository.TankDrive.resetRoughTerrainState();
						
						incrementStep("Done going over rough terrain");
					}
				}
				else if(RoutineDefense == 5) // Rock Wall
				{
					if(Repository.TankDrive.autoRockWall(true, true, 0))
					{
						Repository.TankDrive.resetRockWallState();
						
						incrementStep("Done going over rock wall");
					}
				}
			}
			else if(currentStep == 2) // Routine Position
			{
				if(RoutineDefense == 1 && RoutinePosition != 1) // Low Bar is always Position 1 and Defense 1. So why wouldn't they equal? Abort...
				{
					abort("RoutineDefense and RoutinePosition must be equal if low bar!");
				}
				else if(RoutineGoal == 3) // Don't move farther if we are just spitting the ball out
				{
					incrementStep("Planning on spitting ball out. Not moving forward");
				}
				else if(RoutineDefense == 1) // From Low Bar
				{
					if(Repository.TankDrive.autoMoveDist(1, TankDrive.LOW_BAR_DIST_INCHES))
					{
						Repository.TankDrive.resetAutoMove();
						incrementStep("Done moving forward from low bar");
					}
				}
				else if(RoutinePosition == 2) // From Position 2
				{
					if(Repository.TankDrive.autoMoveDist(1, TankDrive.SECOND_DEFENSE_DIST_INCHES))
					{
						Repository.TankDrive.resetAutoMove();
						incrementStep("Done moving forward from second position");
					}
				}
				else if(RoutinePosition == 3) // From Position 3, but we shoot from the courtyard so let's not move
				{
					incrementStep("Not moving forward, staying in courtyard for third position");
				}
				else if(RoutinePosition == 4) // From Position 4
				{
					if(Repository.TankDrive.autoMoveDist(1, TankDrive.FOURTH_DEFENSE_DIST_INCHES))
					{
						Repository.TankDrive.resetAutoMove();
						incrementStep("Done moving forward from fourth position");
					}
				}
				else if(RoutinePosition == 5) // From Position 5
				{
					if(Repository.TankDrive.autoMoveDist(1, TankDrive.FIFTH_DEFENSE_DIST_INCHES))
					{
						Repository.TankDrive.resetAutoMove();
						incrementStep("Done moving forward from fifth position");
					}
				}
			}
			else if(currentStep == 3) // Routine Goal
			{	
				if(RoutineGoal == 0) // Do nothing? Abort
				{
					abort("No value set for shooting a goal");
				}
				else if(RoutineGoal == 3) // Spit Ball Out
				{
					Repository.TankDrive.goToAngle(180);
					
					if(Repository.TankDrive.isAtAngleSetpoint() && Repository.Shooter.shooterIsAtSetpoint())
					{
						Repository.Shooter.setBallPusher(true);
						fired = true;
					}
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
						
						if(RoutinePosition != 3)
							timeoutSeconds = 8;
						
						incrementStep("Done firing the ball");
					}
					
					numLoops++;
				}
				else if(RoutineGoal == 1) // High Goal, larger setpoint
				{
					Repository.Shooter.setShooterSetpoint(85);
				}
				else if(RoutineGoal == 2) // Low Goal, smaller setpoint
				{
					Repository.Shooter.setShooterSetpoint(50);
				}
				else if(RoutineGoal == 3) // Spit Ball out
				{
					Repository.Shooter.setShooterSetpoint(33);
				}
			}
			else if(currentStep == 4) // Routine Finish
			{
				if(RoutineFinish == 0) // Do nothing? Abort
				{
					abort("No value set for finish");
				}
				else if(RoutineFinish == 1) // Neutral zone
				{
					if(RoutinePosition == 3) // From courtyard
					{
						Repository.TankDrive.goToAngle(0);
						Repository.Shooter.setArmSetpoint(0);
						
						if((Repository.TankDrive.isAtAngleSetpoint() || yawInitiallyOnTarget) && (Repository.Shooter.armIsAtSetpoint() || armInitiallyAtSetpoint))
						{
							yawInitiallyOnTarget = true;
							armInitiallyAtSetpoint = true;
							
							if(Repository.TankDrive.autoMoveDist(0.5, 6))
							{
								if(RoutineDefense == 2) // Moat
								{
									if(Repository.TankDrive.autoMoat(false, true, 0))
									{
										incrementStep("Done going back over moat");
									}
								}
								else if(RoutineDefense == 3) // Ramp Parts
								{
									if(Repository.TankDrive.autoRampParts(false, true, 0))
									{
										incrementStep("Done going back over ramp parts");
									}
								}
								else if(RoutineDefense == 4) // Rough Terrain
								{
									if(Repository.TankDrive.autoRoughTerrain(false, true, 0))
									{
										incrementStep("Done going back over rough terrain");
									}
								}
								else if(RoutineDefense == 5) // Rock Wall
								{
									if(Repository.TankDrive.autoRockWall(false, true, 0))
									{
										incrementStep("Done going back over rock wall");
									}
								}
							}
						}
					}
					else // Not from courtyard, going to go backwards towards drivers and back over the defense
					{
						Repository.TankDrive.goToAngle(0);
						 
						if(RoutineDefense == 1)
						{
							Repository.Shooter.setArmSetpoint(Shooter.LOW_BAR_ENC_DIST);
						}
						else
						{
							Repository.Shooter.setArmSetpoint(0);
						}
						
						
						if((Repository.TankDrive.isAtAngleSetpoint() || yawInitiallyOnTarget) && (Repository.Shooter.armIsAtSetpoint() || armInitiallyAtSetpoint))
						{
							yawInitiallyOnTarget = true;
							armInitiallyAtSetpoint = true;
							
							if(RoutineDefense == 1) // Low Bar
							{
								if(Repository.TankDrive.autoLowBar(false, true, 0))
								{
									incrementStep("Done going back thru low bar");
								}
							}
							else if(RoutineDefense == 2) // Moat
							{
								if(Repository.TankDrive.autoMoat(false, true, 0))
								{
									incrementStep("Done going back over moat");
								}
							}
							else if(RoutineDefense == 3) // Ramp Parts
							{
								if(Repository.TankDrive.autoRampParts(false, true, 0))
								{
									incrementStep("Done going back over ramp parts");
								}
							}
							else if(RoutineDefense == 4) // Rough Terrain
							{
								if(Repository.TankDrive.autoRoughTerrain(false, true, 0))
								{
									incrementStep("Done going back over rough terrain");
								}
							}
							else if(RoutineDefense == 5) // Rock Wall
							{
								if(Repository.TankDrive.autoRockWall(false, true, 0))
								{
									incrementStep("Done going back over rock wall");
								}
							}
						}
					}
				}
				else if(RoutineFinish == 2) // Secret Passage
				{
					abort("Secret passage auto not yet implemented!");
				}
			}
			else if(currentStep == 5)
			{
				finished = true;
				return true;
			}
			
			return false;
		}
		catch(Exception ex)
		{
			if(!errored)
			{
				Repository.Logs.error("Unexpected error while executing auto", ex);
				abort();
				errored = true;
			}
			
			return true;
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
		timeoutSeconds = 5;
	}
	
	public static void abort(String driverStationMessage)
	{
		abort();
		Repository.Logs.warning("Reason for abort: " + driverStationMessage);
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
			+ "LBS=" + Repository.TankDrive.getLowBarState() + ", "
			+ "MS=" + Repository.TankDrive.getMoatState() + ", "
			+ "RPS=" + Repository.TankDrive.getRampPartsState() + ", "
			+ "RTS=" + Repository.TankDrive.getRoughTerrainState() + ", "
			+ "RWS=" + Repository.TankDrive.getRockWallState() + ", "
			+ "F=" + fired + ", "
			+ "NL=" + numLoops + ", "
			+ "E=" + errored + ", "
			+ "Time=" + Repository.Timer.get() + "s"
		);
		
		currentStep = 9001;
		finished = true;
		
		Repository.Timer.stop();
	}
	
	private static void incrementStep(String driverStationMessage)
	{
		incrementStep();
		Repository.Logs.debug("Incremented Step (" + (currentStep-1) + " ->" + currentStep +": " + driverStationMessage);
	}

	private static void incrementStep()
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
		
		data = new RoutineData(start, defense, position, goal, finish);
	}
	
	public static RoutineData getRoutineData()
	{
		return data;
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
}