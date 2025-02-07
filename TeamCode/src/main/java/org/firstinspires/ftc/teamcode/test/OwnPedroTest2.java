package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants.LConstants;


/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
@Autonomous
public class OwnPedroTest2 extends OpMode {
    private Follower follower;

    private final Pose startPose = new Pose(24 - 72,24 - 72, Math.toRadians(0));

    private PathChain rect;

    private Telemetry telemetryA;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(rect, true);
        }

        follower.telemetryDebug(telemetryA);
    }

    /**
     * This initializes the Follower and creates the PathChain for the "triangle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        rect = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(24.000 - 72, 24.000 - 72, Point.CARTESIAN),
                                new Point(14.684 - 72, 109.895    - 72, Point.CARTESIAN),
                                new Point(24.000-72, 120.000-72, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(24.000-72, 120.000-72, Point.CARTESIAN),
                                new Point(27.711-72, 134.053-72, Point.CARTESIAN),
                                new Point(120.000-72, 120.000-72, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(120.000-72, 120.000-72, Point.CARTESIAN),
                                new Point(139.737-72, 112.974-72, Point.CARTESIAN),
                                new Point(120.000-72, 24.000-72, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(120.000-72, 24.000-72, Point.CARTESIAN),
                                new Point(112.500-72, 5.211-72, Point.CARTESIAN),
                                new Point(21.789-72, 8.526-72, Point.CARTESIAN),
                                new Point(24.000-72, 24.000-72, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(360))
                .build();

        follower.followPath(rect);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly triangular shape,"
                + "starting on the bottom-middle point. So, make sure you have enough "
                + "space to the left, front, and right to run the OpMode.");
        telemetryA.update();
    }

}
