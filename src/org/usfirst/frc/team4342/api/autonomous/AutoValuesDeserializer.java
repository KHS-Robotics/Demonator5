package org.usfirst.frc.team4342.api.autonomous;

import java.io.File;
import java.io.FileInputStream;
import java.io.ObjectInputStream;

import org.usfirst.frc.team4342.robot.Repository;

public class AutoValuesDeserializer 
{
	private AutoValuesDeserializer() {}
	
	private static Deserializer deserializer = new Deserializer(null);
	
	public static RoutineData load()
	{
		if(deserializer.isFinished())
		{
			deserializer.run();
		}
		
		return deserializer.getData();
	}
	
	private static class Deserializer
	{
		private static boolean done = true;
		
		private RoutineData data;
		
		public Deserializer(RoutineData data)
		{
			this.data = data;
		}
		
		public RoutineData getData()
		{
			return data;
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
					Repository.Logs.warning("No default auto values set!");
					done = true;
					return;
				}
				
				RoutineData d = null;
		        FileInputStream fileIn = new FileInputStream(file);
		        ObjectInputStream in = new ObjectInputStream(fileIn);
		        d = (RoutineData) in.readObject();
		        in.close();
		        fileIn.close();

		        data = d;
		        
		        Repository.Log.info("Deserialized auto routine data: " + data.toString());
		        
		        done = true;
			}
			catch(Exception ex)
			{
				Repository.Logs.error("Failed to deserialize auto routine data", ex);
				data = null;
				done = true;
			}
		}
	}
}
