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
import org.firstinspires.ftc.teamcode.hardware.Commands.Limelight;
import org.firstinspires.ftc.teamcode.hardware.Commands.Outtake;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.FConstants2;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.LConstants;


@Autonomous(name = "6+0", group = "auto")
public class Specimen6 extends OpMode {
    Intake intake;
    Horizontal_Lift horlift;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;

    private  Pose startPose = new Pose(0, 0, Math.toRadians(0));

    MultipleTelemetry telemetryA;
    private  Pose scorePose = new Pose(29.5, 1.6, Math.toRadians(0));
    private Pose controlPose = new Pose(-4,-17, 0);
    private Pose P1 = new Pose(11.6, -41, 0);
    private Pose P2 = new Pose(11.6, -51, 0);
    private Pose P3 = new Pose(37.8,-34.8, Math.toRadians(270));
    private Pose hp3 = new Pose(11.6, -34, 0);
    private Pose take2 = new Pose(1, -33, 0);
    private Pose take3 = new Pose(0, -33, 0);
    private Pose take4 = new Pose(0, -33, 0);
    private Pose take5 = new Pose(0, -33, 0);
    private Pose take6 = new Pose(0, -33, 0);
    private Pose score2 = new Pose(32.5, 5, 0);
    private Pose score3 = new Pose(32.5, 7, 0);
    private Pose score4 = new Pose(32.5, 10.5, 0);
    private Pose score5 = new Pose(32.5, 12, 0);
    private Pose score6 = new Pose(32.5, 12, 0);
    private Pose park = new Pose(2.7, -30, 0);

    private Pose pick_center = new Pose(2.7, -30, 0);

    private Path scorePreload;
    private PathChain parking, pick1, pick2, pick3, pick32, scoring2, scoring3, scoring4, scoring5, taking2, taking3, taking4, taking5;
    private  PathChain pickingall;
    public void buildPaths() {

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pick1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pick_center), new Point(P1)))
                .setLinearHeadingInterpolation(pick_center.getHeading(), P1.getHeading())
                .build();
        pick2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(P1), new Point(P2)))
                .setLinearHeadingInterpolation(P1.getHeading(), P2.getHeading())
                .build();
        pick3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(P2), new Point(P3)))
                .setLinearHeadingInterpolation(P2.getHeading(), P3.getHeading())
                .build();
        pick32 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(P3), new Point(hp3)))
                .setLinearHeadingInterpolation(P3.getHeading(), hp3.getHeading())
                .build();

        taking2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hp3), new Point(hp3.getX()+5, hp3.getY()), new Point(take2.getX() + 1.0, take2.getY())))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        taking3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score2),new Point(take3.getX()+15, take3.getY()-3), new Point(take3.getX() + 1.5, take3.getY())))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        taking4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score3),new Point(take4.getX()+15, take4.getY()-3), new Point(take4.getX() + 1.5, take4.getY())))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();
        taking5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score4),new Point(take5.getX()+15, take5.getY()-3), new Point(take5.getX() + 1.5, take5.getY())))
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
        parking = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score4),new Point(take5.getX()+15, take5.getY()-3), new Point(take5)))
                .setLinearHeadingInterpolation(hp3.getHeading(), take2.getHeading())
                .build();


    }

    ElapsedTime timer;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                follower.followPath(scorePreload);
                setPathState(1);
                lift.set_target_position(100);
                //intake.vision();
                outtake.setautospec();
                intake.setmidpovishe_take();
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    sleep(100);
                    lift.setpower(0.8);
                    sleep(250);
                    horlift.open();
                    outtake.release();
                    sleep(100);
                    lift.hold_position();
                    lift.set_target_position(0);
                    outtake.setPered_take();


                    intake.vision();
                    intake.open();
                    Pose tarpose = pick_center;
                    follower.holdPoint(tarpose);
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();
                    sleep(500);
                    while(!limelight.isDetected()) {
                        if(timer.milliseconds() > 2000) break;
                        PathChain pathChain = follower.pathBuilder()
                                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(follower.getPose().getX(), follower.getPose().getY()+2)))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build();
                        follower.setMaxPower(0.8);
                        follower.followPath(pathChain, true);
                        sleep(500);
                    }
                    Pose cur = follower.getPose();

                    follower.holdPoint(cur);
                    sleep(200);
                    Pose target = getsamplepose(follower.getPose());
                    intake.rotate_auto(limelight.get_angle());
                    double distance = Math.sqrt(Math.pow(target.getX()-cur.getX(), 2) + Math.pow(target.getY()-cur.getY(), 2));
                    PathChain pathChain = follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(follower.getPose()), new Point(target)))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .build();

                    follower.setMaxPower(0.7);
                    follower.followPath(pathChain, true);
                    sleep(300);
                    intake.open();
                    follower.holdPoint(target);
                    //follower.holdPoint(new Pose(target.getX(), target.getY(), Math.toRadians(-90.0)));
                    intake.setsample_take();
                    sleep(300);

                    intake.close();
                    sleep(100);
                    follower.setMaxPower(1.0);

                    horlift.close();
                    peredacha();
                    outtake.sethb();
                    follower.followPath(pick1);
                    setPathState(2);
                    break;
                }
            case 2:
                if (!follower.isBusy()) {
                    follower.holdPoint(P1);
                    horlift.open();
                    intake.setmidpovishe_take();
                    sleep(700);
                    taking();
                    horlift.close();
                    peredacha();
                    outtake.release();
                    follower.setMaxPower(0.6);
                    follower.followPath(pick2);
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                    follower.holdPoint(P2);
                    horlift.open();
                    intake.setmidpovishe_take();
                    sleep(700);
                    taking();
                    horlift.close();
                    peredacha();
                    outtake.release();
                    follower.setMaxPower(1.0);
                    follower.followPath(pick3);
                    setPathState(4);
                }
            case 4:
                if (!follower.isBusy()) {
                    follower.holdPoint(P3);
                    horlift.open();
                    intake.setmidpovishe_take();
                    intake.rotate_auto(90);
                    sleep(700);
                    taking();
                    horlift.close();
                    peredacha();
                    outtake.release();
                    follower.followPath(pick32);
                    setPathState(-5);
                }
            case -5:
                if (!follower.isBusy()) {
                    follower.holdPoint(hp3);
                    sleep(200);
                    horlift.close();
                    outtake.release();
                    sleep(200);
                    follower.followPath(taking2);
                    setPathState(5);
                }
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    follower.holdPoint(new Pose(take2.getX()-1, take2.getY(), take2.getHeading()));
                    sleep(300);
                    outtake.grab();
                    sleep(200);
                    follower.setMaxPower(1);
                    lift.set_target_position(100);
                    outtake.setautospec();
                    follower.followPath(scoring2);
                    intake.setauto();
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    follower.holdPoint(score2);
                    sleep(100);
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
                if(follower.getPose().getX() <= 1 && follower.getPose().getY() > -37) {

                    outtake.grab();
                    sleep(100);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.holdPoint(take3);
                    follower.setMaxPower(1);
                    sleep(200);
                    lift.set_target_position(100);
                    outtake.setautospec();
                    follower.followPath(scoring3);
                    setPathState(8);

                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    follower.holdPoint(score3);
                    sleep(100);
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
                if(follower.getPose().getX() <= 1 && follower.getPose().getY() > -37) {

                    outtake.grab();
                    sleep(100);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.holdPoint(take4);
                    follower.setMaxPower(1);
                    sleep(200);
                    lift.set_target_position(100);
                    outtake.setautospec();
                    follower.followPath(scoring4);
                    setPathState(10);

                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    follower.holdPoint(score4);
                    sleep(100);
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
                if(follower.getPose().getX() <= 1 && follower.getPose().getY() > -37) {

                    outtake.grab();
                    sleep(100);
                }
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.holdPoint(take5);
                    follower.setMaxPower(1);
                    sleep(200);
                    lift.set_target_position(100);
                    outtake.setautospec();
                    follower.followPath(scoring5);
                    setPathState(12);
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.holdPoint(score5);
                    sleep(100);
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
                    setPathState(-1);
                }
                break;
        }

    }
    public static double adder = 1, pixelstoinches = 0.018;
    public Pose getsamplepose(Pose cur) {
        return new Pose(cur.getX()-limelight.get_y_d(), cur.getY()+limelight.get_x_d()+adder, Math.toRadians(0));

    }

    Limelight limelight;

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void taking () {
        intake.setsample_take();
        sleep(200);
        intake.open();
        intake.close();
        sleep(200);
        intake.setmidpovishe_take();
        outtake.setPered_take();
    }
    public void peredacha() {
        outtake.mayat_up();
        intake.open_chut();
        sleep(100);
        intake.rotate_mid();
        outtake.setPered_take();
        intake.setperedacha();
        sleep(100);
        intake.close();
        sleep(200);
        outtake.mayat_up1();
        outtake.grab();
        sleep(300);
        //lift.set_to_high_basket();
        intake.setmidpovishe_take();
        intake.open();
        outtake.sethb();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot
        limelight.update_camera();
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
        limelight = new Limelight(hardwareMap, telemetry);
        limelight.switch_pipeline(3);
        limelight.start_camera();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        follower.setMaxPower(1.0);
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
            limelight.update_camera();
        }
    }
}

