package org.usfirst.frc.team4342.api.vision;

import java.io.File;
import java.sql.Blob;
import java.util.ArrayList;
import java.util.Iterator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

//import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * 
 * @author Elijah Kaufman
 * @version 1.0
 * @description Uses opencv and network table 3.0 to detect the vision targets
 *
 */
public class Hello {

	/**
	 * static method to load opencv and networkTables
	 */
	static{ 
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		//NetworkTable.setClientMode();
		//NetworkTable.setIPAddress("roborio-4342.local");
	}
	public static final Scalar 
	LOWER_BOUNDS = new Scalar(150,165,240),
	UPPER_BOUNDS = new Scalar(180,190,255);

	
//	the size for resing the image
	public static final Size resize = new Size(1080,960);
	
//	ignore these
	public static VideoCapture videoCapture;
	public static Mat matOriginal, matRGB, matThresh, clusters, matHeirarchy;
	
	public static final int TOP_TARGET_HEIGHT = 97;
	public static final int TOP_CAMERA_HEIGHT = 46;

	public static final double VERTICAL_FOV  = 54;
	public static final double HORIZONTAL_FOV  = 54;
	public static final double CAMERA_ANGLE = 0;
	

	public static void main(String[] args) {
		matOriginal = new Mat();
		matRGB = new Mat();
		matThresh = new Mat();
		clusters = new Mat();
		matHeirarchy = new Mat();
		//NetworkTable table = NetworkTable.getTable("SmartDashboard");
//		while(shouldRun){
//			try {
////				opens up the camera stream and tries to load it
//				videoCapture = new VideoCapture();
//				videoCapture.open("http://10.##./.##.11/mjpg/video.mjpg");
////				Example
////				cap.open("http://10.30.19.11/mjpg/video.mjpg");
////				wait until it is opened
//				while(!videoCapture.isOpened()){}
////				time to actually process the acquired images
//				processImage();
//			} catch (Exception e) {
//				e.printStackTrace();
//				break;
//			}
//		}
		//videoCapture.release();
		//System.exit(0);
		processImage();
	}
	/**
	 * 
	 * reads an image from a live image capture and outputs information to the SmartDashboard or a file
	 */
	public static void processImage(){
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		double x,y,targetAngle,targetY,distance,azimuth;
//		frame counter
		int FrameCount = 0;
		long before = System.currentTimeMillis();
//		only run for the specified time
		while(FrameCount < 100){
			contours.clear();
//			capture from the axis camera
			//videoCapture.read(matOriginal);
//			captures from a static file for testing
			matOriginal = Imgcodecs.imread("C:\\Users\\Magnus\\Desktop\\walrus.png");
			Imgproc.cvtColor(matOriginal,matRGB,Imgproc.COLOR_BGR2RGB);			
			Core.inRange(matRGB, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
			Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, 
					Imgproc.CHAIN_APPROX_SIMPLE);
//			make sure the contours that are detected are at least 20x20 
//			pixels with an area of 400 and an aspect ration greater then 1
			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				Rect rec = Imgproc.boundingRect(matOfPoint);
					if(rec.height < 25 || rec.width < 25){
						iterator.remove();
					continue;
					}
					float aspect = (float)rec.width/(float)rec.height;
					System.out.println(aspect);
					if(aspect < 1.4 || aspect > 1.8) //should be around 1.6ish methinks
						iterator.remove();
				}
				for(MatOfPoint mop : contours){
					Rect rec = Imgproc.boundingRect(mop);
					Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), new Scalar(255,255,255));
			}
			if(contours.size() == 1){
				Rect rec = Imgproc.boundingRect(contours.get(0));
				y = rec.br().y + rec.height / 2;
				y= -((2 * (y / matOriginal.height())) - 1);
				distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) / 
						Math.tan((y * (VERTICAL_FOV / 2.0) + CAMERA_ANGLE) * Math.PI / 180);
				targetAngle = rec.tl().x + rec.width / 2;
				targetAngle = (2 * (targetAngle / matOriginal.width())) - 1;
				azimuth = normalize360(targetAngle*HORIZONTAL_FOV /2.0 + 0);
				System.out.println(distance);
				System.out.println(y*(VERTICAL_FOV/2));
				
			break;
			}
			File f=new File("C:\\Users\\Magnus\\Desktop\\output.png");
			Imgcodecs.imwrite(f.getPath(), matOriginal);
			

			FrameCount++;
			
		}
	}
	/**
	 * @param angle a nonnormalized angle
	 */
	public static double normalize360(double angle){
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