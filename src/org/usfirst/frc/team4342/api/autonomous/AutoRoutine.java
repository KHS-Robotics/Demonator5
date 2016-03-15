package org.usfirst.frc.team4342.api.autonomous;

public enum AutoRoutine 
{
	Unknown(-1),
	RampParts(0),
	RoughTerrain(1),
	Moat(2),
	LowBar(3),
	LowBarAndShoot(4),
	RockWall(5);
	
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
