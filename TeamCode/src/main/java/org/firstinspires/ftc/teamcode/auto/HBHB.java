package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.WebCamera.Detection;
import org.firstinspires.ftc.teamcode.hardware.Commands.Horizontal_Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Intake;
import org.firstinspires.ftc.teamcode.hardware.Commands.Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Limelight;
import org.firstinspires.ftc.teamcode.hardware.Commands.Outtake;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.FConstants2;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "HbHB", group = "auto")
public class HBHB extends OpMode {
    Intake intake;
    Horizontal_Lift horlift;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;

    private  Pose startPose = new Pose(0, 0, Math.toRadians(0));

    MultipleTelemetry telemetryA;
    private  Pose scorePose = new Pose(2.4, 29, Math.toRadians(-45));
    private  Pose pick1pose = new Pose(25, 20, Math.toRadians(0));
    private  Pose pick2pose = new Pose(25, 29, Math.toRadians(0));
    private  Pose pick3pose = new Pose(22, 27.7, Math.toRadians(30));
    private  Pose park = new Pose(58, -3, Math.toRadians(270));
    private  Pose pickfromcenterpose1 = new Pose(53, 4, Math.toRadians(270));
    private  Pose pickfromcenterpose2 = new Pose(58, 4, Math.toRadians(270));
    private  Pose pickfromcenterpose3 = new Pose(63, 4, Math.toRadians(270));
    private PathChain scorePreload, pick1, pick2, pick3, pick4, pick5, pick6, scoring1, scoring2, scoring3, scoring4, scoring5, scoring6, parking;
    private  PathChain pickingall;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose.getX(), scorePose.getY())))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        pick1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pick1pose.getX()-6, pick1pose.getY())))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pick1pose.getHeading() - Math.toRadians(15))
                .build();
        scoring1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pick1pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pick1pose.getHeading(), scorePose.getHeading())
                .build();
        pick2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pick2pose.getX()-6, pick2pose.getY())))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pick2pose.getHeading() - Math.toRadians(15))
                .build();
        scoring2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pick2pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pick2pose.getHeading(), scorePose.getHeading())
                .build();
        pick3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pick3pose.getX()-5, pick3pose.getY())))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pick3pose.getHeading() - Math.toRadians(7))
                .build();
        scoring3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pick3pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pick3pose.getHeading(), scorePose.getHeading())
                .build();
        pick4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickfromcenterpose1.getX(), pickfromcenterpose1.getY()+20), new Point(pickfromcenterpose1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickfromcenterpose1.getHeading() + Math.toRadians(15))
                .build();
        scoring4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickfromcenterpose1), new Point(pickfromcenterpose1.getX(), pickfromcenterpose1.getY()+20), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickfromcenterpose1.getHeading(), scorePose.getHeading())
                .build();
        pick5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickfromcenterpose2.getX(), pickfromcenterpose2.getY()+20), new Point(pickfromcenterpose2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickfromcenterpose2.getHeading() + Math.toRadians(15))
                .build();
        scoring5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickfromcenterpose2), new Point(pickfromcenterpose2.getX(), pickfromcenterpose2.getY()+20), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickfromcenterpose2.getHeading(), scorePose.getHeading())
                .build();
        pick6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickfromcenterpose3.getX(), pickfromcenterpose3.getY()+20), new Point(pickfromcenterpose3)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickfromcenterpose2.getHeading() + Math.toRadians(15))
                .build();
        scoring6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickfromcenterpose3), new Point(pickfromcenterpose3.getX(), pickfromcenterpose3.getY()+20), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickfromcenterpose2.getHeading(), scorePose.getHeading())
                .build();
        parking = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(park)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), park.getHeading())
                .build();

    }

    ElapsedTime timer;
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                lift.set_to_high_basket();
                outtake.setautospec();
                outtake.grab();
                intake.setmidpovishe_take();
                intake.open();
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    sleep(500);
                    outtake.sethb();
                    sleep(200);
                    outtake.release();
                    sleep(100);

                    follower.followPath(pick1, true);
                    sleep(500);
                    outtake.mayat_up();
                    sleep(200);
                    lift.set_target_position(0);
                    setPathState(2);
                    break;
                }
            case 2:
                intake.setmidpovishe_take();
                if (!follower.isBusy()) {

                    follower.holdPoint(new Point(pick1pose), pick1pose.getHeading());
                    sleep(400);
                    taking();
                    follower.followPath(scoring1, true);
                    peredacha();
                    lift.set_to_high_basket();
                    setPathState(3);
                    break;
                }
            case 3:
                if (!follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    sleep(400);
                    outtake.sethb();
                    sleep(200);
                    outtake.release();
                    sleep(100);
                    follower.followPath(pick2, true);
                    sleep(500);
                    outtake.mayat_up();
                    sleep(200);
                    lift.set_target_position(0);
                    setPathState(4);
                    break;
                }
            case 4:
                intake.setmidpovishe_take();
                if (!follower.isBusy()) {
                    follower.holdPoint(new Point(pick2pose), pick2pose.getHeading());
                    sleep(400);
                    taking();
                    follower.followPath(scoring2, true);
                    peredacha();
                    lift.set_to_high_basket();
                    setPathState(5);
                    break;
                }
            case 5:
                if (!follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    sleep(400);
                    outtake.sethb();
                    sleep(200);
                    outtake.release();
                    sleep(100);
                    follower.followPath(pick3, true);
                    sleep(500);
                    outtake.mayat_up();
                    sleep(200);
                    lift.set_target_position(0);
                    setPathState(6);
                    horlift.setpos(0.49);
                    intake.rotate_auto(45);
                    break;
                }
            case 6:
                intake.setsample_take();

                if (!follower.isBusy()) {
                    follower.holdPoint(new Point(pick3pose), pick3pose.getHeading());
                    sleep(400);
                    taking();
                    follower.followPath(scoring3, true);
                    horlift.close();
                    sleep(400);
                    peredacha();
                    lift.set_to_high_basket();
                    setPathState(7);
                    break;
                }
            case 7:
                if (!follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    sleep(600);
                    outtake.sethb();
                    sleep(200);
                    outtake.release();
                    sleep(100);
                    follower.followPath(pick4, true);
                    sleep(500);

                    outtake.mayat_up();
                    sleep(200);
                    lift.set_target_position(0);
                    horlift.open();
                    intake.vision();
                    outtake.setPered_take();
                    sleep(800);
                    setPathState(8);
                    break;
                }
            case 8:
                if (!follower.isBusy()) {
                    initvision();
                    intake.vision();
                    Pose tarpose = pickfromcenterpose1;
                    follower.holdPoint(tarpose);
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();
                    sleep(500);
                    while(!detection.detected) {
                        if(timer.milliseconds() > 2000) break;
                        PathChain pathChain = follower.pathBuilder()
                                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(follower.getPose().getX()+4, follower.getPose().getY()-1)))
                                .setConstantHeadingInterpolation(Math.toRadians(-90))
                                .build();
                        follower.setMaxPower(0.8);
                        follower.followPath(pathChain, true);
                        sleep(600);
                    }
                    Pose cur = follower.getPose();

                    follower.holdPoint(cur);
                    sleep(400);
                    Pose target = getsamplepose(follower.getPose());
                    intake.rotate_auto(detection.getheading());
                    double distance = Math.sqrt(Math.pow(target.getX()-cur.getX(), 2) + Math.pow(target.getY()-cur.getY(), 2));
                    PathChain pathChain = follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(follower.getPose()), new Point(target)))
                            .setConstantHeadingInterpolation(Math.toRadians(-90))
                            .build();
                    follower.setMaxPower(0.7);
                    follower.followPath(pathChain, true);

                    intake.open();
                    intake.setmidpovishe_take();
                    sleep(700);
                    follower.holdPoint(target);
                    //follower.holdPoint(new Pose(target.getX(), target.getY(), Math.toRadians(-90.0)));
                    intake.setsample_take();
                    sleep(300);

                    intake.close();
                    sleep(100);
                    follower.setMaxPower(1.0);
                    follower.followPath(scoring4, true);
                    horlift.close();
                    peredacha();
                    lift.set_to_high_basket();
                    outtake.setautospec();
                    outtake.sethb();
                    setPathState(9);
                    break;
                }
            case 9:
                if (!follower.isBusy()) {
                    follower.holdPoint(scorePose);

                    sleep(200);
                    outtake.release();
                    sleep(100);
                    follower.followPath(pick5, true);
                    sleep(500);

                    outtake.mayat_up();
                    sleep(200);
                    lift.set_target_position(0);
                    horlift.open();
                    intake.vision();
                    sleep(800);
                    setPathState(10);
                    break;
                }
            case 10:
                if (!follower.isBusy()) {
                    intake.vision();
                    Pose tarpose = pickfromcenterpose2;
                    follower.holdPoint(tarpose);
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();
                    sleep(500);
                    while(!detection.detected) {
                        if(timer.milliseconds() > 2000) break;
                        PathChain pathChain = follower.pathBuilder()
                                .addPath(new BezierCurve(new Point(follower.getPose()), new Point(follower.getPose().getX()+4, follower.getPose().getY()-1)))
                                .setConstantHeadingInterpolation(Math.toRadians(-90))
                                .build();
                        follower.setMaxPower(0.8);
                        follower.followPath(pathChain, true);
                        sleep(500);
                    }
                    Pose cur = follower.getPose();

                    follower.holdPoint(cur);
                    sleep(400);
                    Pose target = getsamplepose(follower.getPose());
                    intake.rotate_auto(detection.getheading());
                    double distance = Math.sqrt(Math.pow(target.getX()-cur.getX(), 2) + Math.pow(target.getY()-cur.getY(), 2));
                    PathChain pathChain = follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(follower.getPose()), new Point(target)))
                            .setConstantHeadingInterpolation(Math.toRadians(-90))
                            .build();
                    follower.setMaxPower(0.7);
                    follower.followPath(pathChain, true);

                    intake.open();
                    intake.setmidpovishe_take();
                    sleep(700);
                    follower.holdPoint(target);
                    //follower.holdPoint(new Pose(target.getX(), target.getY(), Math.toRadians(-90.0)));
                    intake.setsample_take();
                    sleep(300);

                    intake.close();
                    sleep(100);
                    follower.setMaxPower(1.0);
                    follower.followPath(scoring5, true);
                    horlift.close();
                    peredacha();
                    lift.set_to_high_basket();
                    outtake.setautospec();
                    outtake.sethb();
                    setPathState(11);
                    break;
                }
            case 11:
                if (!follower.isBusy()) {
                    follower.holdPoint(scorePose);

                    sleep(100);
                    outtake.release();
                    sleep(500);
                    follower.followPath(parking, true);
                    outtake.mayat_up();
                    sleep(200);
                    lift.set_target_position(200);
                    horlift.close();
                    intake.vision();
                    sleep(800);
                    setPathState(12);
                    break;
                }
        }

    }
    public static double adder = 1, pixelstoinches = 0.018;
    public Pose getsamplepose(Pose cur) {
        return new Pose(cur.getX()-detection.getXDist() * pixelstoinches, cur.getY()+detection.getYDist() * pixelstoinches+adder, Math.toRadians(-90));

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    Detection detection;
    public void initvision() {
        detection = new Detection(hardwareMap, telemetry);
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
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        intake.rotate_mid();
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
            lift.update_pid();
        }
    }
    public void sleepandholdlift(int mil) {
        ElapsedTime timersleep = new ElapsedTime();
        timersleep.reset();
        while(timersleep.milliseconds() <= mil) {
            follower.update();
            lift.hold_position();
        }
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
        lift.set_to_high_basket();
        intake.setmidpovishe_take();
        intake.open();
        outtake.setautospec();
    }
}

