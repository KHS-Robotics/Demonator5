 
package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.IterativeRobot;

import org.usfirst.frc.team4342.api.drive.TankDrive;
import org.usfirst.frc.team4342.api.logging.SmartDashboardUpdater;
import org.usfirst.frc.team4342.api.pnuematics.Compressor;
import org.usfirst.frc.team4342.api.shooter.Shooter;

/**
 * FRC Team 4342 (Kennett High School Demon Robotics) Robot Code for Stronghold.
 * 
 * @author Brice Chapman
 * @author Ernie Wilson
 * @author Katie Schuetz
 * @author Payton DuLong
 * @author Shakti Das
 */
public class Robot extends IterativeRobot 
{
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	@Override
    public void robotInit() 
    {
		Repository.initializeAll();
		
		SmartDashboardUpdater.startUpdating(Repository.Log, Repository.ConsoleLog);
		
		Compressor.startAutomaticMode(Repository.Compressor, Repository.PressureSwitch);
    }
	
	/**
	 * This function is run when the robot is first starting autonomous
	 */
	@Override
    public void autonomousInit() 
    {
    	Repository.DriveTrain.setBrakeMode();
    }

    /**
     * This function is called periodically during autonomous
     */
	@Override
    public void autonomousPeriodic() 
    {
    	
    }
	
	/**
	 * This function is run when the robot is first starting operator control
	 */
	@Override
    public void teleopInit()
    {
		Repository.DriveTrain.setBrakeMode();
		
    	TankDrive.startAutomaticMode(
    		Repository.DriveStick,
    		Repository.DriveTrain,
    		Repository.Navx,
    		Repository.Shifter,
    		1
    	);
    	
    	Shooter.startAutomaticMode(
			Repository.ShooterStick, 
			Repository.Accumulator,
			Repository.RightShooter, 
			Repository.LeftShooter, 
			Repository.LoaderX,
			Repository.LoaderY,
			1
		);
    }

    /**
     * This function is called periodically during operator control
     */
	@Override
    public void teleopPeriodic() 
    {
		
    }
    
	/**
	 * This function is called when the robot is first starting disabled
	 */
    @Override
    public void disabledInit()
    {
    	Repository.DriveTrain.setCoastMode();
    	TankDrive.stopAutomaticMode();
    	Shooter.stopAutomaticMode();
    }
    
    /**
     * This function is called periodically during disabled
     */
    @Override
    public void disabledPeriodic()
    {
    	
    }
}

