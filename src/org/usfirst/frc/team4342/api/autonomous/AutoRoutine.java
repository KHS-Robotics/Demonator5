package org.usfirst.frc.team4342.api.autonomous;

public enum AutoRoutine 
{
	Unknown(-1);
	
	private final int id;
	
	private AutoRoutine(int id) 
	{
		this.id = id;
	}
	
	public int getId() 
	{
		return id;
	}
}
