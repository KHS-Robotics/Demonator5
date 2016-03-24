package org.usfirst.frc.team4342.api.autonomous;

import java.io.File;
import java.io.FileOutputStream;
import java.io.ObjectOutputStream;

import org.usfirst.frc.team4342.robot.Repository;

public class AutoValuesSerializer
{
	private AutoValuesSerializer() {}
	
	private static Serializer serializer = new Serializer(null);
	
	public static void save(int routineStart, int routineDefense, int routinePosition, int routineGoal, int routineFinish)
	{
		if(serializer.isFinished())
		{
			serializer.setData(new RoutineData(routineStart, routineDefense, routinePosition, routineGoal, routineFinish));
			serializer.run();
		}
	}
	
	private static class Serializer
	{
		private static boolean done = true;
		
		private RoutineData data;
		
		public Serializer(RoutineData data)
		{
			this.data = data;
		}
		
		public void setData(RoutineData data)
		{
			this.data = data;
		}
		
		public boolean isFinished()
		{
			return done;
		}
		
		public void run()
		{
			try
			{
				done = false;
				
				final String PATH = "/home/lvuser/AutoRoutineData.ser";
				File file = new File(PATH);
				
				if(!file.exists())
				{
					file.createNewFile();
				}
				else
				{
					file.delete();
					file.createNewFile();
				}
				
				FileOutputStream fileOut = new FileOutputStream(file);
		        ObjectOutputStream out = new ObjectOutputStream(fileOut);
		        out.writeObject(data);
		        out.close();
		        fileOut.close();
		        
		        Repository.Log.info("Serialized auto routine data: " + data.toString());
		        
		        done = true;
			}
			catch(Exception ex)
			{
				Repository.Logs.error("Failed to serialize auto routine data", ex);
				done = true;
			}
		}
	}
}
