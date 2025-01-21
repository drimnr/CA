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
public class Lift_PID_test extends LinearOpMode {
    Lift lift;
    @Override
    public void runOpMode() {
        lift = new Lift(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        lift.set_target_position(0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            lift.update_pid();
            lift.tel();
        }
    }}
