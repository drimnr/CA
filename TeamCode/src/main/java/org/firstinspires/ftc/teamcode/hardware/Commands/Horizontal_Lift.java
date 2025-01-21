package org.firstinspires.ftc.teamcode.hardware.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Horizontal_Lift {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    Servo l, r;
    private boolean horliftInitialized = false;

    public static double open = 0.37;
    public static double close = 0;

    public Horizontal_Lift(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        initialize();
    }

    public void initialize(){
        try {
            l = hardwareMap.get(Servo.class, "hor_l");
            r = hardwareMap.get(Servo.class, "hor_r");
            horliftInitialized = true;
            telemetry.addData("intake", "Initialized");
        } catch (IllegalArgumentException iae){
            telemetry.addData("intake", iae.getMessage());
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.update();
    }
    public void open() {
        if (horliftInitialized) {
            l.setPosition(open);
            r.setPosition(1-open);
        }
    }
    public void mid() {
        if (horliftInitialized) {
            l.setPosition((open + close) / 2);
            r.setPosition((1-open + 1 - close) / 2);
        }
    }
    public void close() {
        if (horliftInitialized) {
            l.setPosition(close);
            r.setPosition(1-close);
        }
    }
}