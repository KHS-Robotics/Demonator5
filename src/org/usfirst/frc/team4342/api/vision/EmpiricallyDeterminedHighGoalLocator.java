package org.usfirst.frc.team4342.api.vision;

import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

/**
 * 
 * @author Elijah Kaufman
 * @version 1.0
 * @description Uses opencv and network table 3.0 to detect the vision targets
 *
 */
public class EmpiricallyDeterminedHighGoalLocator 
{
	/**
	 * static method to load opencv
	 */
	static
	{ 
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		//NetworkTable.setClientMode();
		//NetworkTable.setIPAddress("roborio-4342.local");
	}
	
	public static final Scalar LOWER_BOUNDS = new Scalar(100,60,150), UPPER_BOUNDS = new Scalar(110,100,240);

	
	//the size for resing the image
	public static final Size resize = new Size(320,240);
	
	//ignore these
	public static VideoCapture videoCapture;
	public static Mat matOriginal, matHSV, matThresh, clusters, matHeirarchy;
	
	//size in real life: 5.75*2 in
	public static final int TOP_TARGET_HEIGHT = 67;
	public static final int TOP_CAMERA_HEIGHT = 35;

	public static final double VERTICAL_FOV  = 51;
	public static final double HORIZONTAL_FOV  = 49;
	public static final double CAMERA_ANGLE = -2;
	
	public static GoalInfo getDistanceAndAngle(String imgLoc, String ipLoc) 
	{
		matOriginal = new Mat();
		matHSV = new Mat();
		matThresh = new Mat();
		clusters = new Mat();
		matHeirarchy = new Mat();
		
		//NetworkTable table = NetworkTable.getTable("SmartDashboard");
		
		while(true)
		{
			try 
			{
				//opens up the camera stream and tries to load it
				videoCapture = new VideoCapture();
				//videoCapture.open("http://10.0.0.69/mjpg/video.mjpg");
				videoCapture.open(ipLoc);
				
				//Example
				//cap.open("http://10.30.19.11/mjpg/video.mjpg");
				//wait until it is opened
				
				while(!videoCapture.isOpened())
				{
					// wait...
				}
				
				System.out.println("connected to cam");
				
				//time to actually process the acquired images
				return processImage(imgLoc);
			} 
			catch (Exception e) 
			{
				e.printStackTrace();
				break;
			}
		}
		
		//videoCapture.release();
		//System.exit(0);
		
		return processImage(imgLoc);
	}
	
	/**
	 * reads an image from a live image capture and outputs information to the SmartDashboard or a file
	 */
	public static GoalInfo processImage(String imgLoc)
	{
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		double x, y, targetAngle, targetY, distance, azimuth;
		
		//frame counter
		int FrameCount = 0;
		
		while(FrameCount < 100)
		{
			contours.clear();
			
			//capture from the axis camera
			videoCapture.read(matOriginal);
			
			//captures from a static file for testing
			//matOriginal = Imgcodecs.imread("C:\\Users\\Magnus\\Desktop\\walrus.png");
			
			Imgproc.cvtColor(matOriginal,matHSV,Imgproc.COLOR_BGR2HSV);			
			Core.inRange(matHSV, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
			Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
			
			//make sure the contours that are detected are at least 20x20 
			//pixels with an area of 400 and an aspect ration greater then 1
			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) 
			{
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				Rect rec = Imgproc.boundingRect(matOfPoint);
				
				//if(rec.height < 2 || rec.width < 1){
					//iterator.remove();
					//continue;
				//}
				
				float aspect = (float)rec.width/(float)rec.height;
				
				if(aspect < 2.5 || aspect > 3.4) //should be around 2.875 ish methinks
					iterator.remove();
			}
			
			for(MatOfPoint mop : contours)
			{
				Rect rec = Imgproc.boundingRect(mop);
				Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), new Scalar(255,255,255));
				
				//System.out.println("rectangle found check outputMask");
				//File f = new File("C:\\Users\\Magnus\\Desktop\\outputMask.png");
				File f = new File(imgLoc);
				
				Imgcodecs.imwrite(f.getPath(), matOriginal);
			}
			
			if(contours.size() == 1)
			{
				Rect rec = Imgproc.boundingRect(contours.get(0));
				
				y = rec.br().y + rec.height / 2;
				y= -((2 * (y / matOriginal.height())) - 1);
				
				distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) / Math.tan((y * (VERTICAL_FOV / 2.0) + CAMERA_ANGLE) * Math.PI / 180);
				targetAngle = rec.tl().x + rec.width / 2;
				targetAngle = (2 * (targetAngle / matOriginal.width())) - 1;
				azimuth = normalize360(targetAngle*HORIZONTAL_FOV /2.0 + 0);
				
				//System.out.println(distance);
				//System.out.println(y*(VERTICAL_FOV/2));
				
				return new GoalInfo(distance, azimuth);
			}
			
			FrameCount++;
		}
		
		return null;
	}
	
	/**
	 * Normalizes an angle so it's between 0 and 360
	 * @param angle a nonnormalized angle
	 */
	private static double normalize360(double angle)
	{
		while(angle >= 360.0)
        {
            angle -= 360.0;
        }
		
        while(angle < 0.0)
        {
            angle += 360.0;
        }
        
        return angle;
	}
}