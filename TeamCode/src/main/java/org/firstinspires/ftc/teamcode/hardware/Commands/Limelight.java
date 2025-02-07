package org.firstinspires.ftc.teamcode.hardware.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
public class Limelight {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public static double pixels_to_inches= 0.0125;
    private Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        initialize();
    }

    public void initialize() {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            telemetry.setMsTransmissionInterval(11);

            limelight.pipelineSwitch(0);

            /*
             * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
             */
            limelight.start();

            telemetry.addData(">", "Robot Ready.  Press Play.");
        } catch (IllegalArgumentException iae) {
            telemetry.addData("Limelight", iae.getMessage());
        }
    }
    public void switch_pipeline(int n) {
        limelight.pipelineSwitch(n);
    }
    public void start_camera() {
        limelight.start();
    }
    public void stop_camera() {
        limelight.stop();
    }
    double[] ans = null;
    public void update_camera() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        ans = result.getPythonOutput();
    }
    public boolean isDetected() {
        if(ans[0] != 0)
            return true;
        return false;
    }
    public double get_w() {
        return ans[4];
    }
    public double get_h() {
        return ans[5];
    }
    public double get_angle() {
        return ans[3];
    }

    public double get_x_d() {
        return ans[1]*pixels_to_inches;
    }
    public double get_y_d() {
        return ans[2]*pixels_to_inches;
    }
}