package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeamElementDetection.Detection;
import org.firstinspires.ftc.teamcode.hardware.Commands.Horizontal_Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Intake;
import org.firstinspires.ftc.teamcode.hardware.Commands.Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.MecanumBase;
import org.firstinspires.ftc.teamcode.hardware.Commands.Outtake;

@TeleOp

public class Avtonavodka extends LinearOpMode {
    PIDController pid;
    PIDController pidlat;
    Detection detection;
    Intake intake;
    Horizontal_Lift hor_lift;
    public static double kp = 0, ki = 0, kd = 0;

    public static double kp2 = 0, ki2 = 0, kd2 = 0;
    public static double ax = 0;
    public static double lx = 0;
    @Override
    public void runOpMode() {
        pid = new PIDController(kp, ki, kd);
        pidlat = new PIDController(kp2, ki2, kd2);
        detection = new Detection(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        hor_lift = new Horizontal_Lift(hardwareMap, telemetry);

        waitForStart();
        hor_lift.close();
        intake.setperedacha();
        double current_pose = 0;


        while (opModeIsActive()) {
            if(detection.detected){
                intake.rotate_auto(detection.getheading());
                intake.setperedacha();
                if (detection.getYDist() < -70 && current_pose < 0.33)
                    current_pose+=0.005;
                if (detection.getYDist() > 30 && current_pose > 0)
                    current_pose-=0.005;
                if(Math.abs(detection.getYDist()) < 50) {
                    intake.open();
                    intake.setsample_take();
                    sleep(1000);
                    intake.close();
                    sleep(1000);
                    intake.setperedacha();
                    sleep(100000);
                }
                hor_lift.setpos(current_pose);
                ElapsedTime timer = new ElapsedTime();
                sleep(10);
                hor_lift.setpos(current_pose);
            }
        }
    }
}