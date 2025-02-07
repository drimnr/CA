package org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants2 {
    static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.PINPOINT;

        // We can change the value of any variable/constant of FollowerConstants.
        FollowerConstants.mass = 14; // In kg

        FollowerConstants.leftFrontMotorName = "lfd";
        FollowerConstants.leftRearMotorName = "lbd";
        FollowerConstants.rightFrontMotorName = "rfd";
        FollowerConstants.rightRearMotorName = "rbd";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.xMovement = 86.74;

        FollowerConstants.yMovement = 66.34;
        FollowerConstants.forwardZeroPowerAcceleration = -27.5;
        FollowerConstants.lateralZeroPowerAcceleration = -65;


        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15,0.0002,0.0005,0);

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2.5,0.00004,0.01,0);

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.015,0.00001,0.00001,0.6,0);
        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
