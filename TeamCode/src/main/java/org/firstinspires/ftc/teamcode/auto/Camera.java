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
@Autonomous(name = "Camera auto test", group = "auto")
public class Camera extends OpMode {
    Intake intake;
    Horizontal_Lift horlift;
    Detection detection;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;

    private  Pose startPose = new Pose(0, 0, Math.toRadians(270));

    MultipleTelemetry telemetryA;



    ElapsedTime timer;
    public void autonomousPathUpdate() {

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public static double adder = 1, pixelstoinches = 0.018;
    @Override
    public void loop() {
        horlift.open();
        intake.vision();
        // These loop the movements of the robot
        follower.update();
        Pose cur = follower.getPose();
        Pose target = getsamplepose(cur);
        if(detection.detected) {
            sleep(200);
            intake.rotate_auto(detection.getheading());
            double distance = Math.sqrt(Math.pow(target.getX()-cur.getX(), 2) + Math.pow(target.getY()-cur.getY(), 2));
                PathChain pathChain = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(cur), new Point(target)))
                        .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
                        .build();
                follower.setMaxPower(0.7);
                follower.followPath(pathChain, true);

            intake.open();
            intake.setsample_take();
                sleep(700);
            follower.holdPoint(target);
            sleep(100);
            follower.setMaxPower(1.0);
            intake.close();
            sleep(100);
            intake.vision();
            sleep(800);


            sleep(2000);
        }
        else {
            return;
        }
        // Feedback to Driver Hub
        telemetryA.addData("x", cur.getX());
        telemetryA.addData("y", cur.getY());
        telemetryA.addData("tx", target.getX());
        telemetryA.addData("ty", target.getY());
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
        horlift.open();
        outtake.grab();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
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
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
    public Pose getsamplepose(Pose cur) {
        return new Pose(cur.getX()-detection.getXDist() * pixelstoinches, cur.getY()+detection.getYDist() * pixelstoinches+adder, Math.toRadians(270));
    }
    public void sleep(int mil) {
        ElapsedTime timersleep = new ElapsedTime();
        timersleep.reset();
        while(timersleep.milliseconds() <= mil) {
            follower.update();
        }
    }
}

