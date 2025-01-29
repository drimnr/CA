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
        FollowerConstants.mass = 12.8; // In kg

        FollowerConstants.leftFrontMotorName = "lfd";
        FollowerConstants.leftRearMotorName = "lbd";
        FollowerConstants.rightFrontMotorName = "rfd";
        FollowerConstants.rightRearMotorName = "rbd";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    }
}
