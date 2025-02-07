package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Commands.MecanumBase;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.LConstants;

@Config
@TeleOp
@Disabled
public class Gamepad_test extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumBase m = new MecanumBase(hardwareMap, telemetry);
        Constants.setConstants(FConstants.class, LConstants.class);
        PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
        boolean mode = false;
        waitForStart();
        while (opModeIsActive()) {
            poseUpdater.update();
            if(gamepad1.b) {
                mode = true;
            }
            mode = m.field_centric(gamepad1, poseUpdater.getPose().getHeading(), mode);

            telemetry.addData("left y: ", mode);
            telemetry.update();
        }
    }}
