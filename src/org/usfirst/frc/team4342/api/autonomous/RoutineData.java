package org.usfirst.frc.team4342.api.autonomous;

import java.io.Serializable;

public class RoutineData implements Serializable
{
	private int start, defense, position, goal, finish;
	
	public RoutineData(int start, int defense, int position, int goal, int finish)
	{
		this.start = start;
		this.defense = defense;
		this.position = position;
		this.goal = goal;
		this.finish = finish;
	}
	
	public int getStart()
	{
		return start;
	}
	
	public int getDefense()
	{
		return defense;
	}
	
	public int getPosition()
	{
		return position;
	}
	
	public int getGoal()
	{
		return goal;
	}
	
	public int getFinish()
	{
		return finish;
	}
}
