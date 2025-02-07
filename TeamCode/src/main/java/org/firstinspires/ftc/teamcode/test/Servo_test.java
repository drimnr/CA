package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp
@Disabled
public class Servo_test extends LinearOpMode {
    Servo s;
    public static double pos = 0.5;
    @Override
    public void runOpMode() {
        s = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                s.setPosition(pos);
            }
            telemetry.addData("Position: ", s.getPosition());
            telemetry.update();
        }
    }}
