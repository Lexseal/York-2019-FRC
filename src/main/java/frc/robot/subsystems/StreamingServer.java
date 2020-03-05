package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class StreamingServer extends Thread{
    public void run() {
    	CameraServer server = CameraServer.getInstance();
    	UsbCamera camera = server.startAutomaticCapture();
        camera.setResolution(320, 240);
        //camera.setExposureManual(50);
        //camera.setBrightness(50);
        //camera.setWhiteBalanceManual(3500);
        camera.setFPS(30);
        
        CvSink cvSink = server.getVideo();
        CvSource stream = server.putVideo("stream", 320, 240);
        
        Mat img = new Mat();
    	while(!Thread.interrupted()) {
            cvSink.grabFrame(img);
            if (img.width() == 0) {
            	continue;
            }
            stream.putFrame(img);
    	}
    }
}