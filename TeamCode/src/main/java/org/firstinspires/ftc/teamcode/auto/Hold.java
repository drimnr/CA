package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeamElementDetection.Detection;
import org.firstinspires.ftc.teamcode.hardware.Commands.Horizontal_Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Intake;
import org.firstinspires.ftc.teamcode.hardware.Commands.Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Outtake;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.FConstants2;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "hold", group = "auto")
public class Hold extends OpMode {
    Intake intake;
    Horizontal_Lift horlift;
    Detection detection;
    private Follower follower;

    private  Pose startPose = new Pose(0, 0, Math.toRadians(270));

    MultipleTelemetry telemetryA;



    ElapsedTime timer;
    public static double adder = 1, pixelstoinches = 0.018;
    @Override
    public void loop() {
        follower.update();
        Pose cur = follower.getPose();
        telemetryA.addData("x", cur.getX());
        telemetryA.addData("y", cur.getY());
        telemetryA.addData("heading", cur.getHeading());
        telemetryA.update();
        follower.telemetryDebug(telemetryA);
    }

    Lift lift;
    Outtake outtake;
    @Override
    public void init() {
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        horlift = new Horizontal_Lift(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        detection = new Detection(hardwareMap, telemetry);
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake.vision();
        horlift.close();
        outtake.grab();
        timer = new ElapsedTime();
        Constants.setConstants(FConstants2.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.holdPoint(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        follower.holdPoint(startPose);
    }

    @Override
    public void stop() {
    }
    public Pose getsamplepose(Pose cur) {
        return new Pose(cur.getX()-detection.getXDist() * pixelstoinches, cur.getY()+detection.getXDist() * pixelstoinches+adder, startPose.getHeading());
    }
    public void sleep(int mil) {
        ElapsedTime timersleep = new ElapsedTime();
        timersleep.reset();
        while(timersleep.milliseconds() <= mil) {
            follower.update();
        }
    }
}

