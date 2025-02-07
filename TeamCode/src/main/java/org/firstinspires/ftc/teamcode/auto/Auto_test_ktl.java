package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
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
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.LConstants;


@Autonomous(name = "Auto Basket", group = "auto")
public class Auto_test_ktl extends OpMode {
    Intake intake;
    Horizontal_Lift horlift;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;

    private  Pose startPose = new Pose(0, 0, Math.toRadians(0));


    private  Pose scorePose = new Pose(33.8, 1.3, Math.toRadians(0));
    private Pose controlPose = new Pose(0,-17, 0);
    private Pose P1 = new Pose(58.6 , -35.3, 0);
    private Pose controlPose1 = new Pose(55, -55, 0);
    private Pose hp1 = new Pose(11.2, -38.6, 0);
    private Pose controlPose2 = new Pose(53,-34.2, 0);
    private Pose P2 = new Pose(54.5, -49.5, 0);
    private Pose hp2 = new Pose(15, -49.3, 0);
    private Pose controlPose3 = new Pose(48.8, -44.3, 0);
    private Pose P3 = new Pose(53.3,-55.8, 0);
    private Pose hp3 = new Pose(13.6, -54.8, 0);
    private Pose take2 = new Pose(2.7, -30, 0);
    private Pose take3 = new Pose(2.7, -30, 0);
    private Pose take4 = new Pose(2.7, -30, 0);
    private Pose take5 = new Pose(2.7, -30, 0);
    private Pose score2 = new Pose(34, 8.3, 0);
    private Pose score3 = new Pose(34, 8.3, 0);
    private Pose score4 = new Pose(34, 8.3, 0);
    private Pose score5 = new Pose(34, 8.3, 0);
    private Pose park = new Pose(2.7, -30, 0);



    private Path scorePreload, parking;
    private PathChain pick1, pick2, pick3, scoring2, scoring3, scoring4, scoring5, taking2, taking3, taking4, taking5;
    private  PathChain pickingall;
    public void buildPaths() {

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pickingall = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(controlPose), new Point(P1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), P1.getHeading())
                .addPath(new BezierCurve(new Point(P1), new Point(controlPose1), new Point(hp1)))
                .setLinearHeadingInterpolation(P1.getHeading(), hp1.getHeading())



                .addPath(new BezierCurve(new Point(hp1), new Point(controlPose2), new Point(P2)))
                .setLinearHeadingInterpolation(hp2.getHeading(), P2.getHeading())
                .addPath(new BezierCurve(new Point(P2), new Point(hp2)))
                .setLinearHeadingInterpolation(P1.getHeading(), hp1.getHeading())

                .addPath(new BezierCurve(new Point(hp2), new Point(controlPose3), new Point(P3)))
                .setLinearHeadingInterpolation(hp2.getHeading(), P3.getHeading())
                .addPath(new BezierCurve(new Point(P3), new Point(hp3)))
                .setLinearHeadingInterpolation(P1.getHeading(), hp1.getHeading())
                .build();
        pick1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(controlPose), new Point(P1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), P1.getHeading())
                .addPath(new BezierCurve(new Point(P1), new Point(controlPose1), new Point(hp1)))
                .setLinearHeadingInterpolation(P1.getHeading(), hp1.getHeading())
                .build();
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pick2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hp1), new Point(controlPose2), new Point(P2)))
                .setLinearHeadingInterpolation(hp2.getHeading(), P2.getHeading())
                .addPath(new BezierCurve(new Point(P2), new Point(hp2)))
                .setLinearHeadingInterpolation(P1.getHeading(), hp1.getHeading())
                .build();

        pick3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hp2), new Point(controlPose3), new Point(P3)))
                .setLinearHeadingInterpolation(hp2.getHeading(), P3.getHeading())
                .addPath(new BezierCurve(new Point(P3), new Point(hp3)))
                .setLinearHeadingInterpolation(P1.getHeading(), hp1.getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        taking2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hp3), new Point(take2)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        taking3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2), new Point(take3)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        taking4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3), new Point(take4)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        taking5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score4), new Point(take5)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        scoring2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(take2), new Point(score2)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        scoring3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(take3), new Point(score3)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        scoring4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(take4), new Point(score4)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        scoring5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(take5), new Point(score5)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();


    }

    ElapsedTime timer;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                //Бека осы жерге мән бер
                if (!follower.isBusy()) {
                    sleep(1000);
                    follower.followPath(pickingall, true);
                    setPathState(4);
                    break;
                }
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    sleep(1000);
                    follower.followPath(pick2, true);
                    setPathState(3);
                }
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */


                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    sleep(1000);
                    follower.followPath(pick3, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    sleep(1000);
                    follower.followPath(taking2, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    sleep(1000);
                    follower.followPath(scoring2, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    sleep(1000);
                    follower.followPath(taking3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    sleep(1000);
                    follower.followPath(scoring3, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    sleep(1000);
                    follower.followPath(taking4, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    sleep(1000);
                    follower.followPath(scoring4, true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    sleep(1000);
                    follower.followPath(taking5, true);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    sleep(1000);
                    follower.followPath(scoring5, true);
                    setPathState(-1);
                }
                break;
        }

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        lift.update_pid();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    Detection detection;
    Lift lift;
    Outtake outtake;
    @Override
    public void init() {
        lift = new Lift(hardwareMap, telemetry);
        detection = new Detection(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        horlift = new Horizontal_Lift(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        intake.vision();
        horlift.close();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        timer = new ElapsedTime();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.7);
        buildPaths();
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
    public double multiplier = 0.0075;
    public Pose getsampleposition(Pose pose) {
        return new Pose(pose.getX() - detection.getYDist() * multiplier, pose.getY() - detection.getXDist() * multiplier, pose.getHeading());
    }
    public void sleep(int mil) {
        try{
            Thread.sleep(mil);
        }
        catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

