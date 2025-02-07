package org.firstinspires.ftc.teamcode.hardware.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    Servo arm, wrist, rotate, claw;
    private boolean intakeInitialized = false;

    public static double
            open = 0.25,
            close = 0.67;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        initialize();
    }

    public void initialize() {
        try {
            arm = hardwareMap.get(Servo.class, "arm");
            wrist = hardwareMap.get(Servo.class, "wrist");
            rotate = hardwareMap.get(Servo.class, "rotate");
            claw = hardwareMap.get(Servo.class, "claw");
            intakeInitialized = true;
            telemetry.addData("intake", "Initialized");
        } catch (IllegalArgumentException iae) {
            telemetry.addData("intake", iae.getMessage());
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.update();
    }

    public void open() {
        if (intakeInitialized) {
            claw.setPosition(open);
        }
    }

    public static double openchut = 0.57;

    public void open_chut() {
        if (intakeInitialized) {
            claw.setPosition(openchut);
        }
    }

    public void close() {
        if (intakeInitialized) {
            claw.setPosition(close);
        }
    }

    public static double mid = 0.5;
    public static double left = 0.8;
    public static double right = 0.25;
    public void rotate_auto(double angle) {
        if (intakeInitialized) {
            rotate.setPosition(mid - angle/300);
        }
    }
    public void rotate_mid() {
        if (intakeInitialized) {
            rotate.setPosition(mid);
        }
    }

    public void rotate_left() {
        if (intakeInitialized) {
            rotate.setPosition(left);
        }
    }

    public void rotate_right() {
        if (intakeInitialized) {
            rotate.setPosition(right);
        }
    }

    public static double arm_up = 0.5;
    public static double arm_mid = 0.2;
    public static double arm_down = 0.1;
    public static double wrist_down = 0.8, wrist_pered = 0.0, wrist_mid = 0.5;

    public void setmid_take() {
        arm.setPosition(arm_mid);
        wrist.setPosition(wrist_down);
        open();
    }

    public void setmidpovishe_take() {
        arm.setPosition(arm_mid);
        wrist.setPosition(wrist_mid);
    }

    public void setsample_take() {
        arm.setPosition(arm_down);
        wrist.setPosition(wrist_down);
    }

    public void setperedacha() {
        arm.setPosition(arm_up);
        wrist.setPosition(wrist_pered);
    }

    public static double auto_rotate = 0.08;
    public void auto_rotate() {
        rotate.setPosition(auto_rotate);
    }
    public static double arm_vision = 0.3, wrist_vision = 0.12;
    public void vision() {
        arm.setPosition(arm_vision);
        wrist.setPosition(wrist_vision);
    }
}