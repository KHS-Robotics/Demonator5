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
 * @author Magnus-Murray
 * @author Elijah Kaufman - gotta give cred he designed originally
 * @version 6.9 ;)
 * @description DETECT ALL THE THINGS (actually just the high goal location)
 *
 */
public class EmpiricallyDeterminedHighGoalLocator {

	static{ 
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		//NetworkTable.setClientMode();
		//NetworkTable.setIPAddress("roborio-4342.local");
	}
	public static final Scalar 
	LOWER_BOUNDS = new Scalar(150,165,240),
	UPPER_BOUNDS = new Scalar(180,190,255);

	
	public static final Size resize = new Size(320,240);
	
	public static VideoCapture videoCapture;
	public static Mat matOriginal, matRGB, matThresh, clusters, matHeirarchy;
	
	public static final int TOP_TARGET_HEIGHT = 97;
	public static final int TOP_CAMERA_HEIGHT = 46;

	/* axis sez: 
	Horizontal angle of view: 67°
	Vertical angle of view: 51°
	*/

	public static final double VERTICAL_FOV  = 51; //
	public static final double HORIZONTAL_FOV  = 67; //frc says 49 degrees works better but idk
	public static final double CAMERA_ANGLE = 0; //adjust to offset pitch of the robot- get from navX plox
	
	public static final String CAMERA_IP = "http://10.43.42.16/mjpg/video.mjpg";
	public static final String OUTPUT_PATH = "C:\\Users\\Magnus\\Desktop\\output.png";
	public static final String INPUT_PATH = "C:\\Users\\Magnus\\Desktop\\input.png";
	

	public static void main(String[] args) {
		matOriginal = new Mat();
		matRGB = new Mat();
		matThresh = new Mat();
		clusters = new Mat();
		matHeirarchy = new Mat();
		//NetworkTable table = NetworkTable.getTable("SmartDashboard");
//		while(true){
//			try {
//				videoCapture = new VideoCapture();
//				videoCapture.open(CAMERA_IP);

//				while(!videoCapture.isOpened()){} //just wait till its opened, try to do more efficiently?
				processImage();
//			} catch (Exception e) {
//				e.printStackTrace();
//				break;
//			}
//		}
		//videoCapture.release();
		//System.exit(0);
	}
	/**
	 * 
	 * contour that mama!
	 */
	public static void processImage(){
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		double x,y,targetAngle,targetY,distance,azimuth;
		int FrameCount = 0;
		while(FrameCount < 100){
			contours.clear();
			//videoCapture.read(matOriginal); //put this back in when we do network table/robotsource
			matOriginal = Imgcodecs.imread(INPUT_PATH);
			Imgproc.cvtColor(matOriginal,matRGB,Imgproc.COLOR_BGR2RGB); //windows native bgr not rgb cuz ther dumb			
			Core.inRange(matRGB, LOWER_BOUNDS, UPPER_BOUNDS, matThresh);
			Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, 
					Imgproc.CHAIN_APPROX_SIMPLE);
					
			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				Rect rec = Imgproc.boundingRect(matOfPoint);
					if(rec.height < 25 || rec.width < 25){ //tune these values? unless we wont ever be far enough away
						iterator.remove();
					continue;
					}
					float aspect = (float)rec.width/(float)rec.height;
					System.out.println(aspect);
					if(aspect < 1.35 || aspect > 1.8) //should be around 1.6ish methinks
						iterator.remove();
				}
				for(MatOfPoint mop : contours){
					Rect rec = Imgproc.boundingRect(mop);
					Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), new Scalar(0,0,0)); //black
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
				
			break;
			}
			//take out once we finish testing the thing
			File f=new File(OUTPUT_PATH);
			Imgcodecs.imwrite(f.getPath(), matOriginal);
			

			FrameCount++;
			
		}
	}
	/**
	 *normalize that mama!
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
