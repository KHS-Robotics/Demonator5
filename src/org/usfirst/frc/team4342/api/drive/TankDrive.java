package org.usfirst.frc.team4342.api.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;

import java.util.HashMap;

import org.usfirst.frc.team4342.api.drive.DefenseState;
import org.usfirst.frc.team4342.api.shooter.Shooter;
import org.usfirst.frc.team4342.robot.Repository;

public class TankDrive
{
	private static double JOYSTICK_SENSITIVITY = 0.08;
	private static final double DEAD_BAND = 0.04;
	
	public static final double BATTER_YAW = 60.0, LOW_BAR_YAW = 40.0, THIRD_POSITION_YAW = 12.68;
	
	// TODO: Distance per pulse on the right and left drive might be at fault, but the robot definitely
	// did not move to these inch numbers
	public static final double LOW_BAR_DIST_INCHES = 100;
	public static final double SECOND_DEFENSE_DIST_INCHES = 120, FOURTH_DEFENSE_DIST_INCHES = 80, FIFTH_DEFENSE_DIST_INCHES = 140;
	
	// Used at SCH, figured keep them here until todo above is cleared. These are just used for ensuring a crossing
	//public static final double SECOND_DEFENSE_DIST_INCHES = 900, FOURTH_DEFENSE_DIST_INCHES = 900, FIFTH_DEFENSE_DIST_INCHES = 900;
	//public static final double LOW_BAR_DIST_INCHES = 900;
	
	public static final double FOURTH_POSITION_YAW = -15.0;
	public static final double MOAT_HACK_DIST = 1500;
	
	private Joystick j;
	private DriveTrain driveTrain;
	private CANTalon fr, fl, mr, ml, rr, rl;
	private AHRS navX;
	private DoubleSolenoid shifter;
	private Encoder encLeft, encRight;
	
	private double offset;
	
	private PIDController angleControl;
	private boolean firstRunGoStraight = true;
	
	private boolean firstRunAutoMoveDist = true, goingForward;
	private double targetEncCounts, yaw;
	
	private DefenseState rampPartsState = DefenseState.APPROACH, roughTerrainState= DefenseState.APPROACH, moatState= DefenseState.APPROACH, lowBarState= DefenseState.APPROACH, rockWallState= DefenseState.APPROACH;
	private double startingPitch, minPitch, maxPitch, lastPitch;
	private boolean loggedRP, loggedRT, loggedM, loggedLB, loggedRW;
	private boolean firstRun = true;
	
	public static final double RampPartsPitch = 17, RoughTerrainPitch = 8, MoatPitch = 8, LowBarPitch = 5, RockWallPitch = 12;
	
	private HashMap<Integer, Double> POVLookupTable;
	
	private double currentSetpoint;
	
	private boolean setHighGearRampRate, setLowGearRampRate;
	
	public TankDrive(Joystick j, DriveTrain talons, AHRS navX, DoubleSolenoid shifter, 
					Encoder encLeft, Encoder encRight)
	{
		this.j = j;
		
		this.driveTrain = talons;
		fr = driveTrain.getFrontRight();
		fl = driveTrain.getFrontLeft();
		mr = driveTrain.getMiddleRight();
		ml = driveTrain.getMiddleLeft();
		rr = driveTrain.getRearRight();
		rl = driveTrain.getRearLeft();
		
		this.navX = navX;
		
		this.shifter = shifter;
		
		this.encRight = encRight;
		this.encLeft = encLeft;
		
		angleControl = new PIDController(DrivePID.Rotational.kP, DrivePID.Rotational.kI, DrivePID.Rotational.kD, navX, driveTrain);
		angleControl.setContinuous();
		angleControl.setInputRange(-180.0, 180.0);
		angleControl.setOutputRange(-1.0, 1.0);
		angleControl.setAbsoluteTolerance(3);
		angleControl.disable();
		
		driveTrain.setPIDController(angleControl);
		
		POVLookupTable = new HashMap<Integer, Double>();
		POVLookupTable.put(0, 0.0);
		POVLookupTable.put(90, BATTER_YAW);
		POVLookupTable.put(180, 180.0);
		POVLookupTable.put(270, -BATTER_YAW);
	}
	
	public synchronized void drive(int shiftButton, int straightButton)
	{
		int pov = j.getPOV();
		if(POVLookupTable.containsKey(pov))
		{
			goToSetpoint(POVLookupTable.get(pov));
			
			goStraight(sensitivityControl(j.getRawAxis(3)-j.getRawAxis(2)));
		}
		else if(j.getRawButton(straightButton))
		{
			if (firstRunGoStraight)
			{
				goToSetpoint(navX.getYaw());
				
				firstRunGoStraight = false;
			}
			else
			{
				goStraight(sensitivityControl(j.getRawAxis(3)-j.getRawAxis(2)));
			}
		}
		else if(j.getRawButton(2))
		{
			goToSetpoint(currentSetpoint + 1);
		}
		else if(j.getRawButton(3))
		{
			goToSetpoint(currentSetpoint - 1);
		}
		else
		{
			turnPIDOff();
			joystickDrive(shiftButton);
			
			firstRunGoStraight = true;
		}
	}
	
	public void joystickDrive(int shiftButton)
	{
		checkUserShift(shiftButton);

		double posy = sensitivityControl(j.getRawAxis(3));
		double negy = -sensitivityControl(j.getRawAxis(2));
		double x = sensitivityControl(j.getRawAxis(0));
		
		double y = posy + negy;
		
		try
		{
			fr.set(y-x);
			fl.set(y+x);
			mr.set(y-x);
			ml.set(y+x);
			rr.set(y-x);
			rl.set(y+x);
		}
		catch (Exception ex)
		{
			Repository.Logs.error("Failed to set drive motors", ex);
		}
	}
	
	public boolean autoRampParts(boolean forward, boolean target, double goalAngle)
	{
		double startAngle = Math.abs(getYaw()) > 90 ? 180 : 0;
		double currentPitch = navX.getPitch();
		double direction = 0.75;
		
		if (rampPartsState == DefenseState.APPROACH)
		{
			if (firstRun)
			{
				driveTrain.setBrakeMode();
				
				startingPitch = navX.getPitch();
				minPitch = navX.getPitch();
				maxPitch = navX.getPitch();
				goToAngle(startAngle);
				firstRun = false;
			}
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(isAtAngleSetpoint())
			{
				rampPartsState = DefenseState.CLIMB;
				minPitch = currentPitch;
				maxPitch = currentPitch;
			}
		}
		else if (rampPartsState == DefenseState.CLIMB)
		{
			if(forward)
				goStraight(direction);
			else
				goStraight(-direction);
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(!forward)
			{
				if (minPitch <= -RampPartsPitch && currentPitch > minPitch)
				{
					rampPartsState = DefenseState.PEAK;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
			else
			{
				if (maxPitch >= RampPartsPitch && currentPitch < lastPitch)
				{
					rampPartsState = DefenseState.PEAK;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
		}
		else if (rampPartsState == DefenseState.PEAK)
		{
			if(target)
				direction /= 2; 
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(!forward)
			{
				if (maxPitch >= RampPartsPitch && currentPitch < lastPitch)
				{
					rampPartsState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
			else
			{
				if (minPitch <= -RampPartsPitch && currentPitch > minPitch)
				{
					rampPartsState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
		}
		else if (rampPartsState == DefenseState.DESCENT)
		{
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if (Math.abs((currentPitch - startingPitch)) < 10 && Math.abs(currentPitch - lastPitch) <= 4)
			{
				rampPartsState = DefenseState.FINISHING;
				minPitch = currentPitch;
				maxPitch = currentPitch;
			}
		}
		else if (rampPartsState == DefenseState.FINISHING)
		{
			
			if(target)
			{
				goToAngle(normalizeYaw(getYaw() - goalAngle));
				
				if (isAtAngleSetpoint())
					rampPartsState = DefenseState.FINISH;
			}
			else
				rampPartsState = DefenseState.FINISH;
			
		}
		else if (rampPartsState == DefenseState.FINISH)
		{
			if(!firstRun)
			{
				stopAll();
				firstRun = true;
			}
			else
			{
				if(!loggedRP)
					Repository.Logs.warning("Need to reset ramp parts routine");
				loggedRP = true;
			}
			
			
			return true;
		}
		
		lastPitch = currentPitch;
		
		return false;
	}
	
	public boolean autoRoughTerrain(boolean forward, boolean target, double goalAngle)
	{
		double startAngle = Math.abs(getYaw()) > 90 ? 180 : 0;
		double currentPitch = navX.getPitch();
		double direction = 0.75;
		
		if (roughTerrainState == DefenseState.APPROACH)
		{
			if (firstRun)
			{
				driveTrain.setBrakeMode();
				
				startingPitch = navX.getPitch();
				minPitch = navX.getPitch();
				maxPitch = navX.getPitch();
				goToAngle(startAngle);
				firstRun = false;
			}
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(isAtAngleSetpoint())
			{
				roughTerrainState = DefenseState.CLIMB;
				minPitch = currentPitch;
				maxPitch = currentPitch;
			}
		}
		else if (roughTerrainState == DefenseState.CLIMB)
		{
			if(forward)
				goStraight(direction);
			else
				goStraight(-direction);
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(!forward)
			{
				if (minPitch <= -RoughTerrainPitch && currentPitch > minPitch)
				{
					roughTerrainState = DefenseState.PEAK;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
			else
			{
				if (maxPitch >= RoughTerrainPitch && currentPitch < lastPitch)
				{
					roughTerrainState = DefenseState.PEAK;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
		}
		else if (roughTerrainState == DefenseState.PEAK)
		{
			if(target)
				direction /= 2;
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(!forward)
			{
				if (maxPitch >= RoughTerrainPitch && currentPitch < lastPitch)
				{
					roughTerrainState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
			else
			{
				if (minPitch <= -RoughTerrainPitch && currentPitch > minPitch)
				{
					roughTerrainState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
		}
		else if (roughTerrainState == DefenseState.DESCENT)
		{
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if (Math.abs((currentPitch - startingPitch)) < 10 && Math.abs(currentPitch - lastPitch) <= 4)
			{
				roughTerrainState = DefenseState.FINISHING;
				minPitch = currentPitch;
				maxPitch = currentPitch;
			}
		}
		else if (roughTerrainState == DefenseState.FINISHING)
		{
			if(target)
			{
				goToAngle(normalizeYaw(getYaw() - goalAngle));
				
				if (isAtAngleSetpoint())
					roughTerrainState = DefenseState.FINISH;
			}
			else
				roughTerrainState = DefenseState.FINISH;
		}
		else if (roughTerrainState == DefenseState.FINISH)
		{
			if(!firstRun)
			{
				stopAll();
				firstRun = true;
			}
			else
			{
				if(!loggedRT)
					Repository.Logs.warning("Need to reset rough terrain routine");
				loggedRT = true;
			}
			
			return true;
		}
		
		lastPitch = currentPitch;
		
		return false;
	}
	
	public boolean autoMoat(boolean forward, boolean target, double goalAngle)
	{
		double startAngle = Math.abs(getYaw()) > 90 ? 180 : 0;
		double currentPitch = navX.getPitch();
		double direction = 0.75;
		
		if (moatState == DefenseState.APPROACH)
		{
			if (firstRun)
			{
				driveTrain.setBrakeMode();
				
				startingPitch = navX.getPitch();
				minPitch = navX.getPitch();
				maxPitch = navX.getPitch();
				goToAngle(startAngle);
				firstRun = false;
			}
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(isAtAngleSetpoint())
			{
				moatState = DefenseState.CLIMB;
				minPitch = currentPitch;
				maxPitch = currentPitch;
			}
		}
		else if (moatState == DefenseState.CLIMB)
		{
			if(forward)
				goStraight(direction);
			else
				goStraight(-direction);
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(!forward)
			{
				if (minPitch <= -MoatPitch && currentPitch > minPitch)
				{
					moatState = DefenseState.PEAK;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
			else
			{
				if (maxPitch >= MoatPitch && currentPitch < lastPitch)
				{
					moatState = DefenseState.PEAK;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
		}
		else if (moatState == DefenseState.PEAK)
		{
			if(target)
				direction /= 2;
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(!forward)
			{
				if (maxPitch >= MoatPitch && currentPitch < lastPitch)
				{
					moatState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
			else
			{
				if (minPitch <= -MoatPitch && currentPitch > minPitch)
				{
					moatState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
		}
		else if (moatState == DefenseState.DESCENT)
		{
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if (Math.abs((currentPitch - startingPitch)) < 10 && Math.abs(currentPitch - lastPitch) <= 4)
			{
				moatState = DefenseState.FINISHING;
				minPitch = currentPitch;
				maxPitch = currentPitch;
			}
		}
		else if (moatState == DefenseState.FINISHING)
		{
			if(target)
			{
				goToAngle(normalizeYaw(getYaw() - goalAngle));
				
				if (isAtAngleSetpoint())
					moatState = DefenseState.FINISH;
			}
			else
				moatState = DefenseState.FINISH;
		}
		else if (roughTerrainState == DefenseState.FINISH)
		{
			if(!firstRun)
			{
				stopAll();
				firstRun = true;
			}
			else
			{
				if(!loggedM)
					Repository.Logs.warning("Need to reset ramp parts routine");
				loggedM = true;
			}
			
			return true;
		}
		
		lastPitch = currentPitch;
		
		return false;
	}
	
	public boolean autoLowBar(boolean forward, boolean target, double goalAngle)
	{
		double startAngle = Math.abs(getYaw()) > 90 ? 180 : 0;
		double currentPitch = navX.getPitch();
		double direction = 0.75;
		
		if (lowBarState == DefenseState.APPROACH)
		{
			if (firstRun)
			{
				driveTrain.setBrakeMode();
				
				Repository.Shooter.setArmSetpoint(Shooter.LOW_BAR_ENC_DIST);
				
				startingPitch = navX.getPitch();
				minPitch = navX.getPitch();
				maxPitch = navX.getPitch();
				goToAngle(startAngle);
				firstRun = false;
			}
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(isAtAngleSetpoint() && Repository.Shooter.armIsAtSetpoint())
			{
				Repository.ArmController.disablePID();
				lowBarState = DefenseState.CLIMB;
				minPitch = currentPitch;
				maxPitch = currentPitch;
			}
		}
		else if (lowBarState == DefenseState.CLIMB)
		{
			if(forward)
				goStraight(direction);
			else
				goStraight(-direction);
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(!forward)
			{
				if (minPitch <= -LowBarPitch && currentPitch > minPitch)
				{
					lowBarState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
			else
			{
				if (maxPitch >= LowBarPitch && currentPitch < lastPitch)
				{
					lowBarState = DefenseState.PEAK;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
		}
		else if (lowBarState == DefenseState.PEAK)
		{
			if(target)
				direction /= 2;
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(!forward)
			{
				if (maxPitch >= LowBarPitch && currentPitch < maxPitch && currentPitch < lastPitch)
				{
					lowBarState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
			else
			{
				if (minPitch <= -LowBarPitch && currentPitch > minPitch)
				{
					lowBarState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
		}
		else if (lowBarState == DefenseState.DESCENT)
		{
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if (Math.abs((currentPitch - startingPitch)) < 10 && Math.abs(currentPitch - lastPitch) <= 4)
			{
				lowBarState = DefenseState.FINISHING;
				minPitch = currentPitch;
				maxPitch = currentPitch;
			}
		}
		else if (lowBarState == DefenseState.FINISHING)
		{
			if(target)
			{
				goToAngle(normalizeYaw(getYaw() - goalAngle));
				
				if (isAtAngleSetpoint())
					lowBarState = DefenseState.FINISH;
			}
			else
				lowBarState = DefenseState.FINISH;
		}
		else if (lowBarState == DefenseState.FINISH)
		{
			if(!firstRun)
			{
				stopAll();
				Repository.ArmController.enablePID();
				firstRun = true;
			}
			else
			{
				if(!loggedLB)
					Repository.Logs.warning("Need to reset ramp parts routine");
				loggedLB = true;
			}
			
			return true;
		}
		
		lastPitch = currentPitch;
		
		return false;
	}
	
	public boolean autoRockWall(boolean forward, boolean target, double goalAngle)
	{
		double startAngle = Math.abs(getYaw()) > 90 ? 180 : 0;
		double currentPitch = navX.getPitch();
		double direction = 0.75;
		
		if (rockWallState == DefenseState.APPROACH)
		{
			if (firstRun)
			{
				driveTrain.setBrakeMode();
				
				startingPitch = navX.getPitch();
				minPitch = navX.getPitch();
				maxPitch = navX.getPitch();
				goToAngle(startAngle);
				firstRun = false;
			}
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(isAtAngleSetpoint())
			{
				rockWallState = DefenseState.CLIMB;
				minPitch = currentPitch;
				maxPitch = currentPitch;
			}
		}
		else if (rockWallState == DefenseState.CLIMB)
		{
			if(forward)
				goStraight(direction);
			else
				goStraight(-direction);
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(!forward)
			{
				if (minPitch <= -RockWallPitch && currentPitch > minPitch)
				{
					rockWallState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
			else
			{
				if (maxPitch >= RockWallPitch && currentPitch < lastPitch)
				{
					rockWallState = DefenseState.PEAK;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
		}
		else if (rockWallState == DefenseState.PEAK)
		{
			if(target)
				direction /= 2; 
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if(!forward)
			{
				if (maxPitch >= RockWallPitch && currentPitch < lastPitch)
				{
					rockWallState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
			else
			{
				if (minPitch <= -RockWallPitch && currentPitch > minPitch)
				{
					rockWallState = DefenseState.DESCENT;
					minPitch = currentPitch;
					maxPitch = currentPitch;
				}
			}
		}
		else if (rockWallState == DefenseState.DESCENT)
		{
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if (Math.abs((currentPitch - startingPitch)) < 10 && Math.abs(currentPitch - lastPitch) <= 4)
			{
				rockWallState = DefenseState.FINISHING;
				minPitch = currentPitch;
				maxPitch = currentPitch;
			}
		}
		else if (rockWallState == DefenseState.FINISHING)
		{
			
			if(target)
			{
				goToAngle(normalizeYaw(getYaw() - goalAngle));
				
				if (isAtAngleSetpoint())
					rockWallState = DefenseState.FINISH;
			}
			else
				rockWallState = DefenseState.FINISH;
			
		}
		else if (rockWallState == DefenseState.FINISH)
		{
			if(!firstRun)
			{
				stopAll();
				firstRun = true;
			}
			else
			{
				if(!loggedRW)
					Repository.Logs.warning("Need to reset ramp parts routine");
				loggedRW = true;
			}
			
			return true;
		}
		
		lastPitch = currentPitch;
		
		return false;
	}
	
	public synchronized boolean autoMoveDist(double output, double inches)
	{
		if(firstRunAutoMoveDist)
		{
			driveTrain.setBrakeMode();
			
			if(output > 0)
			{
				goingForward = true;
				targetEncCounts = getCurrentEncoderDistance() + (2*Math.abs(inches));
			}
			else
			{
				goingForward = false;
				targetEncCounts = getCurrentEncoderDistance() - (2*Math.abs(inches));
			}
			
			yaw = getYaw();
			firstRunAutoMoveDist = false;
		}
		
		if(goingForward)
		{
			if(getCurrentEncoderDistance() >= targetEncCounts)
			{
				stopAll();
				return true;
			}
		}
		else
		{
			if(getCurrentEncoderDistance() <= targetEncCounts)
			{
				stopAll();
				return true;
			}
		}
		
		
		goStraight(output, yaw);
		
		return false;
	}
	
	public synchronized void resetAutoMove()
	{
		firstRunAutoMoveDist = true;
	}
	
	public synchronized double getCurrentEncoderDistance()
	{
		return Math.abs(encLeft.getDistance()) + Math.abs(encRight.getDistance());
	}
	
	public synchronized double getYaw()
	{
        double offsettedValue = (double) (navX.getYaw() - offset);
        return normalizeYaw(offsettedValue);
	}
	
	public double normalizeYaw(double yaw)
	{
		if (yaw < -180)
			yaw += 360;
        if (yaw > 180)
        	yaw -= 360;
        
        return yaw;
	}
	
	public void setYawOffset(double offset)
	{
		this.offset = normalizeYaw(offset);
		
		if(this.offset == offset)
			return;
		
		POVLookupTable.clear();
		
		POVLookupTable.put(0, normalizeYaw(0.0 + offset));
		POVLookupTable.put(90, normalizeYaw(BATTER_YAW + offset));
		POVLookupTable.put(180, normalizeYaw(180.0 + offset));
		POVLookupTable.put(270, normalizeYaw(-BATTER_YAW + offset));
	}
	
	public void resetYawOffset()
	{
		offset = 0;
		
		POVLookupTable.clear();
		
		POVLookupTable.put(0, 0.0);
		POVLookupTable.put(90, 45.0);
		POVLookupTable.put(180, 180.0);
		POVLookupTable.put(270, -45.0);
	}
	
	public double getOffset()
	{
		return offset;
	}
	
	public void resetAutoDefense()
	{
		resetYawOffset();
		loggedRP = loggedRT = loggedM = loggedLB = loggedRW = false;
		rampPartsState = DefenseState.APPROACH;
		roughTerrainState = DefenseState.APPROACH;
		moatState = DefenseState.APPROACH;
		lowBarState = DefenseState.APPROACH;
		rockWallState = DefenseState.APPROACH;
	}
	
	public void resetRampPartsState()
	{
		loggedRP = false;
		rampPartsState = DefenseState.APPROACH;
	}
	
	public DefenseState getRampPartsState()
	{
		return rampPartsState;
	}
	
	public void resetRoughTerrainState()
	{
		loggedRT = false;
		roughTerrainState = DefenseState.APPROACH;
	}
	
	public DefenseState getRoughTerrainState()
	{
		return roughTerrainState;
	}
	
	public void resetMoatState()
	{
		loggedM = false;
		moatState = DefenseState.APPROACH;
	}
	
	public DefenseState getMoatState()
	{
		return moatState;
	}
	
	public void resetLowBarState()
	{
		loggedLB = false;
		lowBarState = DefenseState.APPROACH;
	}
	
	public DefenseState getLowBarState()
	{
		return lowBarState;
	}
	
	public void resetRockWallState()
	{
		loggedRW = false;
		rockWallState = DefenseState.APPROACH;
	}
	
	public DefenseState getRockWallState()
	{
		return rockWallState;
	}
	
	public void goStraight(double direction)
	{
		setDirection(direction);
	}
	
	public void goStraight(double direction, double angle)
	{
		goToSetpoint(angle);
		goStraight(direction);
	}
	
	public void goToAngle(double angle)
	{
		goToSetpoint(angle);
		setDirection(0.0);
	}
	
	public synchronized void stopAll()
	{
		turnPIDOff();
		driveTrain.stopAll();
	}
	
	public synchronized void turnPIDOn()
	{
		angleControl.enable();
	}
	
	public synchronized void turnPIDOff()
	{
		if(angleControl.isEnabled())
			angleControl.disable();
	}
	
	public boolean isAtAngleSetpoint()
	{
		return Math.abs(angleControl.getError()) <= 3;
	}
	
	public synchronized void goToSetpoint(double setpointAngle)
	{
		currentSetpoint = setpointAngle;
		angleControl.setSetpoint(normalizeYaw(setpointAngle + offset));
		turnPIDOn();
	}
	
	public synchronized void setYawPID(double p, double i, double d)
	{
		angleControl.setPID(p, i, d);
	}
	
	public synchronized void setDirection(double direction)
	{
		driveTrain.setDirection(direction);
	}
	
	public synchronized double getDirection()
	{
		return driveTrain.getDirection();
	}
	
	private synchronized void checkUserShift(int button)
	{
		if(j.getRawButton(button))
		{
			if(!setHighGearRampRate)
			{
				driveTrain.setVoltageRampRate(16);
				setHighGearRampRate = true;
				setLowGearRampRate = false;
			}
			
			shifter.set(DoubleSolenoid.Value.kReverse);
		}
		else
		{
			if(!setLowGearRampRate)
			{
				driveTrain.setVoltageRampRate(64);
				setLowGearRampRate = true;
				setHighGearRampRate = false;
			}
			
			shifter.set(DoubleSolenoid.Value.kForward);	
		}
	}
	
	private double sensitivityControl(double input)
	{
		if(Math.abs(input) < DEAD_BAND)
			input = 0;
		return (JOYSTICK_SENSITIVITY*Math.pow(input, 3))+((1-JOYSTICK_SENSITIVITY)*input);
	}
}
