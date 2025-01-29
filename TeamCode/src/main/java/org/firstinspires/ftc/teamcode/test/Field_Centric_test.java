package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Commands.MecanumBase;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.LConstants;

@Config
@TeleOp
public class Field_Centric_test extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumBase m = new MecanumBase(hardwareMap, telemetry);
        Constants.setConstants(FConstants.class, LConstants.class);
        PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
        boolean mode = false;
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) mode = true;
            poseUpdater.update();
            mode = m.field_centric(gamepad1, poseUpdater.getPose().getHeading(), mode);
            telemetry.update();
        }
    }}