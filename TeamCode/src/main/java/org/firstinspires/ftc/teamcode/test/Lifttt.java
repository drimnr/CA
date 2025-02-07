package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Commands.Intake;
import org.firstinspires.ftc.teamcode.hardware.Commands.Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Outtake;

@TeleOp
@Disabled
public class Lifttt extends LinearOpMode {
    Lift lift;
    String mode = "MANUAL";
    @Override
    public void runOpMode() {
        lift = new Lift(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        lift.set_target_position(0);
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (mode == "PID") {
                if (gamepad2.left_stick_y != 0) {
                    mode = "MANUAL";
                }
                lift.update_pid();
            }
            else {
                if (gamepad2.a) {
                    mode = "PID";
                    lift.set_to_high_basket();
                }
                lift.setpower1(gamepad2.left_stick_y);
            }
        }
    }}
