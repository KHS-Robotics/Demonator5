package org.usfirst.frc.team4342.api.drive.joysticks;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is for using an Xbox Controller as a joystick. It is simply a 
 * wrapper for FRC's implementation of a joystick.
 * 
 * @see edu.wpi.first.wpilibj.Joystick
 * 
 * @author Ernie Wilson
 */
public class XboxController 
{
	private Joystick j;
	
	/**
	 * Constructs a wrapper for a joystick for an Xbox Controller
	 * @param port the joystick to wrap the class around
	 */
	public XboxController(int port)
	{
		this.j = new Joystick(port);
	}
	
	/**
	 * Constructs a wrapper for a joystick for an Xbox Controller
	 * @param j the joystick to wrap the class around
	 */
	public XboxController(Joystick j)
	{
		this.j = j;
	}
	
	/**
	 * The green 'A' button
	 * @return true if the 'A' button is being pressed; false otherwise
	 */
	public boolean getButtonA()
	{
		return getRawButton(1);
	}
	
	/**
	 * The red 'B' button
	 * @return true if the 'B' button is being pressed; false otherwise
	 */
	public boolean getButtonB()
	{
		return getRawButton(2);
	}
	
	/**
	 * The blue 'X' button
	 * @return true if the 'X' button is being pressed; false otherwise
	 */
	public boolean getButtonX()
	{
		return getRawButton(3);
	}
	
	/**
	 * The yellow 'Y' button
	 * @return true if the 'Y' button is being pressed; false otherwise
	 */
	public boolean getButtonY()
	{
		return getRawButton(4);
	}
	
	/**
	 * The left bumper (LB) above the left trigger
	 * @return true if the left bumper is being pressed; false otherwise
	 */
	public boolean getLeftBumper()
	{
		return getRawButton(5);
	}
	
	/**
	 * The right bumper (RB) above the right trigger
	 * @return true if the right bumper is being pressed; false otherwise
	 */
	public boolean getRightBumper()
	{
		return getRawButton(6);
	}
	
	/**
	 * The back button to the left of the main X button
	 * @return true if the back button is being pressed; false otherwise
	 */
	public boolean getButtonBack()
	{
		return getRawButton(7);
	}
	
	/**
	 * The start button to the right of the main X button
	 * @return true if the start button is being pressed; false otherwise
	 */
	public boolean getButtonStart()
	{
		return getRawButton(8);
	}
	
	/**
	 * The button on the left thumb stick
	 * @return true if the button on the left thumb stick is being pressed; false otherwise
	 */
	public boolean getButtonLeftStick()
	{
		return getRawButton(9);
	}
	
	/**
	 * The button on the right thumb stick
	 * @return true if the button on the right thumb stick is being pressed; false otherwise
	 */
	public boolean getButtonRightStick()
	{
		return getRawButton(10);
	}
	
	/**
	 * The X-Axis on the left thumb stick
	 * @return the current X position on the left thumb stick from [-1, 1] where 1 is right
	 */
	public double getXLeftStick()
	{
		return getRawAxis(1);
	}
	
	/**
	 * The Y-Axis on the left thumb stick
	 * @return the current Y position on the left thumb stick from [-1, 1] where 1 is up
	 */
	public double getYLeftStick()
	{
		return -getRawAxis(2);
	}
	
	/**
	 * The X-Axis on the right thumb stick
	 * @return the current X position on the right thumb stick from [-1, 1] where 1 is right
	 */
	public double getXRightStick()
	{
		return getRawAxis(4);
	}
	
	/**
	 * The Y-Axis on the right thumb stick
	 * @return the current Y position on the right thumb stick from [-1, 1] where 1 is up
	 */
	public double getYRightStick()
	{
		return -getRawAxis(5);
	}
	
	/**
	 * The Directional Pad on the bottom left
	 * @return the current axis position on the Directional Pad
	 */
	public double getDirectionalPad()
	{
		return getRawAxis(6);
	}
	
	/**
	 * Get the button value (starting at button 1) The appropriate button is returned as a boolean value
	 * @param button the button number to be read (starting at 1).
	 * @return true if the button is being pressed; false otherwise
	 */
	private boolean getRawButton(int button)
	{
		return j.getRawButton(button);
	}
	
	/**
	 * Get the value of the axis
	 * @param axis the axis to read, starting at 0
	 * @return the current value of the axis
	 */
	private double getRawAxis(int axis)
	{
		return j.getRawAxis(axis);
	}
}
