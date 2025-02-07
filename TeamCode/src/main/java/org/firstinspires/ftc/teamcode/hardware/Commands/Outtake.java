package org.firstinspires.ftc.teamcode.hardware.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outtake {
    HardwareMap hardwareMap;
    private Telemetry telemetry;

    Servo mayatr, mayatl, outtake, outtake_arm, rotate;

    private boolean outtakeInitialized = false;

    public static double
            mayatr_up = 0.25,
            mayatr_up1 = 0.15,
            mayatr_down = 1,
            mayatr_mid = 0.56,
            mayatl_up = 0.81,
            mayatl_down = 0.00,
            mayatl_mid = 0.45,
            open = 0.0,
            close = 0.38,
            pered_take = 0.1,
            zad_take = 0.35,
            spec = 0.43,
            hb = 0.65;

    public Outtake(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        initialize();
    }

    public void initialize(){
        try {
            outtake_arm = hardwareMap.get(Servo.class, "outtake_arm");
            outtake = hardwareMap.get(Servo.class, "outtake");
            mayatr = hardwareMap.get(Servo.class, "mayatr");
            mayatl = hardwareMap.get(Servo.class, "mayatl");
            outtakeInitialized = true;
            telemetry.addData("outtake", "Initialized");
        } catch (IllegalArgumentException iae){
            telemetry.addData("outtake", iae.getMessage());
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.update();

    }
    // TAKING POSITIONS

    // GRIP
    public void grab() {
        outtake.setPosition(close);
    }

    public void release() {
        outtake.setPosition(open);
    }
    public static double mayat_spec = 0.32, arm_spec = 0.23;
    public void mayat_specimen() {
        mayatr.setPosition(mayat_spec);
        mayatl.setPosition(arm_spec);
    }
    public void mayat_up() {
        mayatr.setPosition(mayatr_up);
        mayatl.setPosition(mayatl_up);
    }
    public static double mayat_auto_spec = 0.64, arm_auto_spec = 0.43;
    public void setautospec() {
        outtake_arm.setPosition(arm_auto_spec);
        mayatr.setPosition(mayat_auto_spec);
        grab();
    }
    public static double open_chut = 0.33;
    public void open_chut() {
        outtake.setPosition(open_chut);
    }
    public void mayat_up1() {
        mayatr.setPosition(mayatr_up1);
    }
    public void mayat_mid() {
        mayatr.setPosition(mayatr_mid);
        mayatl.setPosition(mayatl_mid);
    }

    public void mayat_down() {
        mayatr.setPosition(mayatr_down);
        mayatl.setPosition(mayatl_down);
    }
    public void setPered_take() {
        outtake_arm.setPosition(pered_take);
        mayat_up();
        release();
    }
    public void setZad_take() {
        outtake_arm.setPosition(zad_take);
        mayat_down();
        release();
    }
    public void sethb() {
        outtake_arm.setPosition(hb);
        mayat_mid();
    }
    public void setspecimen() {
        outtake_arm.setPosition(spec);
        mayat_specimen();
    }

}