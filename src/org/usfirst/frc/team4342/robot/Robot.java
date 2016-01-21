 
package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * FRC Team 4342 (Kennett High School Demon Robotics) Robot Code for Stronghold.
 * 
 * @author Brice Chapman
 * @author Ernie Wilson
 * @author Katie Schuetz
 * @author Payton DuLong
 * @author Shakti Das
 * @author Jake Saltzberg
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
    	NavxSmartDashboradTest();
    }
    /**
     * 
     */
    public void NavxSmartDashboradTest()
    {
    	SmartDashboard.putNumber("Navx Yaw", Repository.Navx.getYaw());
    	SmartDashboard.putNumber("Navx Pitch", Repository.Navx.getPitch());
    	SmartDashboard.putNumber("Navx Roll", Repository.Navx.getRoll());
    	SmartDashboard.putNumber("Navx Accelerometer X", Repository.Navx.getRawAccelX());
    	SmartDashboard.putNumber("Navx Accelerometer Y", Repository.Navx.getRawAccelY());
    	SmartDashboard.putNumber("Navx Accelerometer Z", Repository.Navx.getRawAccelZ());
    	SmartDashboard.putNumber("Navx Mag X", Repository.Navx.getRawMagX());
    	SmartDashboard.putNumber("Navx Mag Y", Repository.Navx.getRawMagY());
    	SmartDashboard.putNumber("Navx Mag Z", Repository.Navx.getRawMagZ());
    	SmartDashboard.putBoolean("Connected", Repository.Navx.isConnected());
    	SmartDashboard.putBoolean("Calibrating", Repository.Navx.isCalibrating());
    }
   
}

