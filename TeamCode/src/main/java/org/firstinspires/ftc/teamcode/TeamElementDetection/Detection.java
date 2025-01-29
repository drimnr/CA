package org.firstinspires.ftc.teamcode.TeamElementDetection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeamElementDetection.Pipeline.OwnPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Detection {
    private OpenCvCamera camera;
    private OwnPipeline ownPipeline;
    private double xDist = 0;
    private double yDist = 0;
    private double heading = 0;
    public boolean detected;

    public Detection(HardwareMap hardwareMap, Telemetry telemetry) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ownPipeline = new OwnPipeline(telemetry, this);

        camera.setPipeline(ownPipeline);
        camera.openCameraDevice();
        camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 30);
    }

    // Getter methods for xDist and yDist
    public void status(boolean detected) {
        this.detected = detected;
    }
    public double getXDist() {
        return xDist;
    }

    public double getYDist() {
        return yDist;
    }
    public double getheading() {
        return heading;
    }

    // Methods to update xDist and yDist from the pipeline
    public void updateDistances(double xDist, double yDist, double heading) {
        this.xDist = xDist;
        this.yDist = yDist;
        this.heading = heading;
    }
    public static Scalar lowerYellow = new Scalar(10, 150, 180);
    public static Scalar upperYellow = new Scalar(30, 255, 255);
    public static Scalar lowerSecondary = new Scalar(90, 50, 50);  // Начальный цвет - синий
    public static Scalar upperSecondary = new Scalar(130, 255, 255);


    public void setSecondaryColor(String color) {
        switch (color.toLowerCase()) {
            case "blue":
                lowerSecondary = new Scalar(90, 50, 50);
                upperSecondary = new Scalar(130, 255, 255);
                break;
            case "red":
                lowerSecondary = new Scalar(0, 150, 150);
                upperSecondary = new Scalar(10, 255, 255);
                break;
            default:
                System.out.println("Invalid color selected.");
                break;
        }
    }

    public Scalar getLowerYellow() {
        return lowerYellow;
    }

    public Scalar getUpperYellow() {
        return upperYellow;
    }

    public Scalar getLowerSecondary() {
        return lowerSecondary;
    }

    public Scalar getUpperSecondary() {
        return upperSecondary;
    }
}
