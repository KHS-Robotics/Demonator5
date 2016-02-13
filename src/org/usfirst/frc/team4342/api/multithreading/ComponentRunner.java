package org.usfirst.frc.team4342.api.multithreading;

public class ComponentRunner 
{
	private ComponentRunner() {}
	
	public static void startAutomaticMode(Component comp)
	{
		if(comp.run)
			return;
		
		comp.run = true;
		
		Thread t = new Thread(comp);
		
		t.start();
	}
	
	public static void stopAutomaticMode(Component comp)
	{
		if(comp.run)
		{
			comp.stopAll();
			comp.run = false;
		}
	}
}
