package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TeamElementDetection.Detection;
import org.firstinspires.ftc.teamcode.hardware.Commands.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.Commands.Horizontal_Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Intake;
import org.firstinspires.ftc.teamcode.hardware.Commands.MecanumBase;

@Config
@TeleOp
public class AutoNavigationIntake extends OpMode
{
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
    public void init() {
        pid = new PIDController(kp, ki, kd);
        pidlat = new PIDController(kp2, ki2, kd2);
        detection = new Detection(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        hor_lift = new Horizontal_Lift(hardwareMap, telemetry);


    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        hor_lift.close();
        intake.setperedacha();
    }
    double current_pose = 0;
    @Override
    public void loop() {
        if(detection.detected){
            intake.rotate_auto(detection.getheading());
            intake.setperedacha();
            if (detection.getYDist() < -50 && current_pose < 0.33)
                current_pose+=0.001;
            if (detection.getYDist() > 50 && current_pose > 0)
                current_pose-=0.001;
            if(Math.abs(detection.getYDist()) < 50) {
                intake.open();
                intake.setsample_take();
                ElapsedTime timer1 = new ElapsedTime();
                while(timer1.milliseconds() > 2000){
                    intake.open();
                    intake.setsample_take();
                }
                intake.close();
                ElapsedTime timer2 = new ElapsedTime();
                while(timer2.milliseconds() > 200)  {
                    intake.close();
                }
                intake.setperedacha();
                ElapsedTime timer3 = new ElapsedTime();
                while(timer2.milliseconds() > 200000)  {
                    intake.close();
                }
            }
            hor_lift.setpos(current_pose);
            ElapsedTime timer = new ElapsedTime();
            while(timer.milliseconds() > 60) hor_lift.setpos(current_pose);
        }

    }


    @Override
    public void stop() {
    }

}
