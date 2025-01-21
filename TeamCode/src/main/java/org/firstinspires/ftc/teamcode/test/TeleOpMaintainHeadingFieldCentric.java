package org.firstinspires.ftc.teamcode.test;
/*
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RoadRunner.PoseStorage;
import org.firstinspires.ftc.teamcode.hardware.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RoadRunner.util.DashboardUtil;

@Config
@TeleOp(name="Align to heading", group = "advanced")
@Disabled
public class TeleOpMaintainHeadingFieldCentric extends LinearOpMode {

    private double targetHeading = 0.0;
    public static double PRESET_HEADING = Math.toRadians(90);
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d poseEstimate = PoseStorage.currentPose;
        drive.getLocalizer().setPoseEstimate(poseEstimate);

        headingController.setInputBounds(-Math.PI, Math.PI);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            poseEstimate = drive.getLocalizer().getPoseEstimate();
            boolean rightStickActive = Math.abs(gamepad1.right_stick_x) > 0.01;

            if (rightStickActive) {
                targetHeading = poseEstimate.getHeading();
            }

            if (gamepad1.a) {
                targetHeading = PRESET_HEADING;
            }

            headingController.setTargetPosition(targetHeading);
            double headingCorrection = headingController.update(poseEstimate.getHeading()) * DriveConstants.kV;

            Vector2d fieldCentricInput = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            Pose2d driveDirection = new Pose2d(
                    fieldCentricInput,
                    rightStickActive ? gamepad1.right_stick_x : headingCorrection
            );

            drive.setWeightedDrivePower(driveDirection);

            drive.getLocalizer().update();

            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.addData("Current Heading (deg)", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("X", poseEstimate.getX());
            telemetry.addData("Y", poseEstimate.getY());
            telemetry.update();
        }
    }
}*/
