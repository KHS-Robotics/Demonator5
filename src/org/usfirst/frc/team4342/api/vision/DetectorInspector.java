package org.usfirst.frc.team4342.api.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class DetectorInspector 
{
	public static Goal getGoal(String imgLoc)
	{
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		
		double[] low = new double[3];
		double[] high = new double[3];
		double[] preV = {0, 0, 0};
		double[] curr = {0, 0, 0};

		Mat matHSV = new Mat();
		Mat mat = Imgcodecs.imread(imgLoc);
		Imgproc.cvtColor(mat, matHSV, Imgproc.COLOR_BGR2HSV);
		
//		VideoCapture videoCapture = new VideoCapture();
//		videoCapture.open("http://10.0.0.69/mjpg/video.mjpg");
//		while(!videoCapture.isOpened()){}
//		videoCapture.read(mat);
		//System.out.println(mat.get(0, 0)[0]+" "+mat.get(0, 0)[1]+" "+mat.get(0, 0)[2]);
		
		preV = matHSV.get(0, 0);
		//System.out.println(preV[0]);
		
		for(int i = 1; i < matHSV.height(); i++)
		{
			for(int j = 1; j < matHSV.width();j++)
			{
				curr=matHSV.get(i, j);
				System.out.println(curr[0]);
				if(curr[0] > preV[0])
				{
					high[0] = curr[0];
				}
				if(curr[1] > preV[1])
				{
					high[1] = curr[1];
				}
				if(curr[2] > preV[2])
				{
					high[2] = curr[2];
				}
				
				preV = curr;
			}
		}
		
		preV = matHSV.get(0, 0);
		
		for(int i = 1; i < matHSV.height(); i++)
		{
			for(int j = 1; j < matHSV.width(); j++)
			{
				curr = matHSV.get(i, j);
				if(curr[0] < preV[0])
				{
					low[0] = curr[0];
				}
				if(curr[1] < preV[1])
				{
					low[1] = curr[1];
				}
				if(curr[2] < preV[2])
				{
					low[2] = curr[2];
				}
				
				preV = curr;
			}
		}
		
		return new Goal(low, high);
	}
}
