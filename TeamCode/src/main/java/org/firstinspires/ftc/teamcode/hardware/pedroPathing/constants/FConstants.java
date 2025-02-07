package org.firstinspires.ftc.teamcode.hardware.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.PINPOINT;

        // We can change the value of any variable/constant of FollowerConstants.
        FollowerConstants.mass = 13; // In kg

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


        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.09,0.0003,0.002,0);

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1,0.0003,0.1,0);

        FollowerConstants.zeroPowerAccelerationMultiplier = 0.8;

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.013,-0.0008,0.00003,0.6,0);
        FollowerConstants.pathEndTimeoutConstraint = 50;
    }
}
