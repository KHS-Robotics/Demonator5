 
package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team4342.robot.components.Repository.Navx;

import org.usfirst.frc.team4342.api.drive.TankDrive;

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
	private TankDrive drive;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	@Override
    public void robotInit() 
    {
		Repository.initializeAll();
		
		drive = new TankDrive(
			Repository.DriveStick,
			Repository.FrontRight,
			Repository.FrontLeft,
			Repository.MiddleRight,
			Repository.MiddleLeft,
			Repository.RearRight,
			Repository.RearLeft,
			Navx
		);
    }
	
	/**
	 * This function is run when the robot is first starting autonomous
	 */
	@Override
    public void autonomousInit() 
    {
    	
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
    	
    }

    /**
     * This function is called periodically during operator control
     */
	@Override
    public void teleopPeriodic() 
    {
        drive.drive();
    }
    
	/**
	 * This function is called when the robot is first starting disabled
	 */
    @Override
    public void disabledInit()
    {
    	
    }
    
    /**
     * This function is called periodically during disabled
     */
    @Override
    public void disabledPeriodic()
    {
    	navxSmartDashboradTest();
    }
    
    public void navxSmartDashboradTest()
    {
    	SmartDashboard.putNumber("Navx-Yaw", Navx.getYaw());
    	SmartDashboard.putNumber("Navx-Pitch", Navx.getPitch());
    	SmartDashboard.putNumber("Navx-Roll", Navx.getRoll());
    	SmartDashboard.putNumber("Navx-Accel=X", Navx.getRawAccelX());
    	SmartDashboard.putNumber("Navx-Accel-Y", Navx.getRawAccelY());
    	SmartDashboard.putNumber("Navx-Accel-Z", Navx.getRawAccelZ());
    	SmartDashboard.putNumber("Navx-Mag-X", Navx.getRawMagX());
    	SmartDashboard.putNumber("Navx-Mag-Y", Navx.getRawMagY());
    	SmartDashboard.putNumber("Navx-Mag-Z", Navx.getRawMagZ());
    	SmartDashboard.putBoolean("Connected", Navx.isConnected());
    	SmartDashboard.putBoolean("Calibrating", Navx.isCalibrating());
    }
   
}

