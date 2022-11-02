package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList; 
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint;


@Autonomous( name = "Howard: Auto" )

public class FishOpMode extends LinearOpMode {

   public DcMotor frontMotor, rightMotor, leftMotor; 
   public OpenCvCamera webcam; 
   public double x, y; 
   public double speed = 0; 
   public double angle = 0; 
   
 
    @Override
    public void runOpMode()
    {
        frontMotor = hardwareMap.get(DcMotor.class, "Motor1" ); 
        rightMotor = hardwareMap.get(DcMotor.class, "Motor2" ); 
        leftMotor = hardwareMap.get(DcMotor.class, "Motor3" ); 
        
        frontMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        rightMotor.setDirection( DcMotorSimple.Direction.FORWARD );
        leftMotor.setDirection( DcMotorSimple.Direction.FORWARD );
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        
        webcam.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming( 1280, 720, OpenCvCameraRotation.UPRIGHT );
            }
            @Override
            public void onError(int errorCode) {}
        });
        
        webcam.setPipeline( new FishPipeline() ); 
        waitForStart();
        
        while( opModeIsActive() )
        {
            this.move( speed, angle ); 
        }
    }
    
    /**
     * Moves the robot. No spinning, no saucing. Just linear-like, across-the-floor
     *  movement.
     * 
     * @param speed the speed the robot will move at
     * @param angle the angle at which the robot will move
     */
    public void move( double speed, double angle ) {
        
        frontMotor.setPower( speed * Math.cos( Math.toRadians( angle + 180  ) ) );
        rightMotor.setPower( speed * Math.cos( Math.toRadians( angle + 60 ) ) );
        leftMotor.setPower( speed * Math.cos( Math.toRadians( angle + 300 ) ) );
    }
    
    class FishPipeline extends OpenCvPipeline
    {
        final double MIDDLE_X = 640;
        final double MIDDLE_Y = 360; 
        Mat fish = new Mat(); 
        
            @Override
            public Mat processFrame( Mat rgba )
            {
                Core.inRange( rgba, new Scalar( 215, 0, 0, 0 ), new Scalar( 255, 165, 135, 255), fish ); 
            
                ArrayList< MatOfPoint > contours = new ArrayList< MatOfPoint >(); 
            
                 Imgproc.findContours( fish, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
             
                if( contours.isEmpty() )
                    return fish;
                
                MatOfPoint largest = contours.get(0);

                for(MatOfPoint contour : contours) {
                    if(Imgproc.contourArea(contour) > Imgproc.contourArea(largest)) {
                    
                    largest = contour;
                }
        
        
                Moments m = Imgproc.moments(largest);
            
                x = m.m10 / m.m00 / rgba.width() - 0.5;
                y = m.m01 / m.m00 / rgba.height() - 0.5;
            
                speed = Math.hypot( ( x - MIDDLE_X ), ( y - MIDDLE_Y)  );
                angle = Math.toDegrees( Math.atan2( (MIDDLE_Y - y), (MIDDLE_X - x ) ) );
                
            }
            
            return fish; 
        }
    }
}
