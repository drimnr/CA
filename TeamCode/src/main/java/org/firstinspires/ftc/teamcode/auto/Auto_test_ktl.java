package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import org.firstinspires.ftc.teamcode.hardware.Commands.Horizontal_Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Intake;
import org.firstinspires.ftc.teamcode.hardware.Commands.Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Outtake;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.FConstants2;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.LConstants;


@Autonomous(name = "Auto test", group = "auto")
public class Auto_test_ktl extends OpMode {
    Intake intake;
    Horizontal_Lift horlift;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;

    private  Pose startPose = new Pose(0, 0, Math.toRadians(0));

    MultipleTelemetry telemetryA;
    private  Pose scorePose = new Pose(33, 1.3, Math.toRadians(0));
    private Pose controlPose = new Pose(-7,-17, 0);
    private Pose P1 = new Pose(53 , -35.3, 0);
    private Pose controlPose1 = new Pose(53, -55, 0);
    private Pose hp1 = new Pose(25, -38.6, 0);
    private Pose controlPose2 = new Pose(53,-34.2, 0);
    private Pose P2 = new Pose(53, -49.5, 0);
    private Pose hp2 = new Pose(25, -49.3, 0);
    private Pose controlPose3 = new Pose(48.8, -44.3, 0);
    private Pose P3 = new Pose(53,-57, 0);
    private Pose hp3 = new Pose(25, -55.8, 0);
    private Pose take2 = new Pose(1, -33, 0);
    private Pose take3 = new Pose(4, -33, 0);
    private Pose take4 = new Pose(4, -33, 0);
    private Pose take5 = new Pose(4, -33, 0);
    private Pose score2 = new Pose(32, 4, 0);
    private Pose score3 = new Pose(32, 5, 0);
    private Pose score4 = new Pose(32, 6, 0);
    private Pose score5 = new Pose(32, 7, 0);
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
                .addPath(new BezierCurve(new Point(P1.getX(), hp1.getY()), new Point(hp1)))
                .setLinearHeadingInterpolation(P1.getHeading(), hp1.getHeading())



                .addPath(new BezierCurve(new Point(hp1), new Point(controlPose2), new Point(P2)))
                .setLinearHeadingInterpolation(hp2.getHeading(), P2.getHeading())
                .addPath(new BezierCurve(new Point(P2), new Point(hp2)))
                .setLinearHeadingInterpolation(P1.getHeading(), hp1.getHeading())

                .addPath(new BezierCurve(new Point(hp2), new Point(controlPose3), new Point(P3)))
                .setLinearHeadingInterpolation(hp2.getHeading(), P3.getHeading())
                .addPath(new BezierCurve(new Point(P3), new Point(hp3)))
                .setLinearHeadingInterpolation(P1.getHeading(), hp1.getHeading())

                .addPath(new BezierLine(new Point(hp3), new Point(take2)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
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
                .addPath(new BezierCurve(new Point(hp3),new Point(take2.getX()+15, take2.getY()-3), new Point(take2)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        taking3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score2),new Point(take3.getX()+15, take3.getY()-3), new Point(take3)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        taking4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score3),new Point(take4.getX()+15, take4.getY()-3), new Point(take4)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        taking5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score4),new Point(take5.getX()+15, take5.getY()-3), new Point(take5)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        scoring2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(take2),new Point(score2.getX()-15, score2.getY()+5), new Point(score2)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        scoring3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(take3),new Point(score3.getX()-15, score3.getY()+5), new Point(score3)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        scoring4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(take4),new Point(score4.getX()-15, score4.getY()+5), new Point(score4)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        scoring5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(take5),new Point(score5.getX()-15, score5.getY()+5), new Point(score5)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();


    }

    ElapsedTime timer;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                lift.set_to_high_chamber();
                outtake.setspecimen();
                break;
            case 1:
                //Бека осы жерге мән бер
                if (!follower.isBusy()) {
                    lift.setpower(0.8);
                    sleep(250);
                    outtake.release();
                    sleep(100);
                    lift.hold_position();
                    lift.set_target_position(0);
                    outtake.setZad_take();

                    follower.followPath(pickingall);
                    setPathState(5);
                    break;
                }
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    sleep(250);
                    follower.followPath(pick2, true);
                    setPathState(3);
                }
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */


                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    sleep(250);
                    follower.followPath(pick3, true);
                    setPathState(4);
                }
                break;
            case 4:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    sleep(250);
                    outtake.release();
                    follower.followPath(taking2);
                    outtake.setZad_take();
                    setPathState(5);
                }
                break;
            case 5:
                if(follower.getPose().getX() <= 3.3 && follower.getPose().getY() > -37) {

                    outtake.grab();
                    sleep(100);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    sleep(200);
                    outtake.setspecimen();
                    lift.set_to_high_chamber();
                    follower.followPath(scoring2);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    lift.setpower(0.8);
                    sleep(250);
                    outtake.release();
                    sleep(100);
                    lift.hold_position();
                    lift.set_target_position(0);
                    outtake.setZad_take();
                    outtake.release();
                    follower.setMaxPower(1.0);
                    sleep(250);
                    follower.followPath(taking3);
                    setPathState(7);

                }
                break;
            case 7:
                if(follower.getPose().getX() <= 3.3 && follower.getPose().getY() > -37) {

                    outtake.grab();
                    sleep(100);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    sleep(200);
                    outtake.setspecimen();
                    lift.set_to_high_chamber();
                    follower.followPath(scoring3);
                    setPathState(8);

                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    lift.setpower(0.8);
                    sleep(250);
                    outtake.release();
                    sleep(100);
                    lift.hold_position();
                    lift.set_target_position(0);
                    outtake.setZad_take();
                    outtake.release();
                    follower.setMaxPower(1.0);
                    sleep(250);
                    follower.followPath(taking4);
                    setPathState(9);

                }
                break;
            case 9:
                if(follower.getPose().getX() <= 3.3 && follower.getPose().getY() > -37) {

                    outtake.grab();
                    sleep(100);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    sleep(200);
                    outtake.setspecimen();
                    lift.set_to_high_chamber();
                    follower.followPath(scoring4);
                    setPathState(10);

                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    lift.setpower(0.8);
                    sleep(250);
                    outtake.release();
                    sleep(100);
                    lift.hold_position();
                    lift.set_target_position(0);
                    outtake.setZad_take();
                    follower.setMaxPower(1.0);
                    outtake.release();
                    sleep(250);
                    follower.followPath(taking5);
                    setPathState(11);

                }
                break;
            case 11:
                if(follower.getPose().getX() <= 3.3 && follower.getPose().getY() > -37) {

                    outtake.grab();
                    sleep(100);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    sleep(200);
                    outtake.setspecimen();
                    lift.set_to_high_chamber();
                    follower.followPath(scoring5);
                    setPathState(12);
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    lift.setpower(0.8);
                    sleep(250);
                    outtake.release();
                    sleep(100);
                    lift.set_target_position(0);
                    outtake.setZad_take();
                    outtake.release();
                    sleep(250);
                    follower.followPath(parking);
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
        telemetryA.addData("path state", pathState);
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
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
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake.setperedacha();
        horlift.close();
        outtake.grab();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        timer = new ElapsedTime();
        Constants.setConstants(FConstants2.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
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
    public void sleep(int mil) {
        ElapsedTime timersleep = new ElapsedTime();
        timersleep.reset();
        while(timersleep.milliseconds() <= mil) {
            follower.update();
        }
    }
}

