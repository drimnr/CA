package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class Gamepad_test extends LinearOpMode {
    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("left y: ", gamepad1.left_stick_y);
            telemetry.addData("left x: ", gamepad1.left_stick_x);
            telemetry.addData("right y: ", gamepad1.right_stick_y);
            telemetry.addData("right x: ", gamepad1.right_stick_x);
            telemetry.update();
        }
    }}
