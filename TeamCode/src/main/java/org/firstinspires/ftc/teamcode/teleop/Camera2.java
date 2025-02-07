package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.WebCamera.Detection;
import org.firstinspires.ftc.teamcode.hardware.Commands.Horizontal_Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Intake;
@Config

@TeleOp

public class Camera2 extends LinearOpMode {
    PIDController pid;
    PIDController pidlat;
    Detection detection;
    Intake intake;
    Horizontal_Lift hor_lift;
    public static double kp = 0, ki = 0, kd = 0;

    public static double kp2 = 0, ki2 = 0, kd2 = 0;
    public static double ax = 0;
    public static double lx = 0;
    public static double multiplier = 0.27/(34/2.54), adder = 0.05, pixelstoinches = 0.018;
    @Override
    public void runOpMode() {
        pid = new PIDController(kp, ki, kd);
        pidlat = new PIDController(kp2, ki2, kd2);
        detection = new Detection(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        hor_lift = new Horizontal_Lift(hardwareMap, telemetry);
        hor_lift.setpos(0.45);
        intake.vision();
        waitForStart();

        double current_pose = 0.33;


        while (opModeIsActive()) {
            intake.rotate_auto(detection.getheading());
            if(gamepad1.a){
                intake.open();
                intake.setsample_take();
                sleep(800);
                intake.close();
                sleep(150);
                intake.vision();
                sleep(1000);
            }
            telemetry.addData("y", detection.getYDist()*pixelstoinches);
            telemetry.addData("x", detection.getXDist()*pixelstoinches);
            sleep(60);
        }
    }
}