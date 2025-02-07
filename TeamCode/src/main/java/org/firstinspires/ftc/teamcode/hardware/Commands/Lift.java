package org.firstinspires.ftc.teamcode.hardware.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public static int high_chamber = 350, high_basket = 2080;
    DcMotor lift_l, lift_r;
    public static double kp = 0.013, kd = 0.1, ki = 0.0001, kf = 0.00002;
    PIDFController pidfController;
    public static double down_power = 0.6, holdp = 0.1;
    private boolean liftInitialized = false;
    public static int target_position = 0, current_position = 0;
    public static String mode = "PID";
    public Lift(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        initialize();
    }

    public void initialize(){
        try {
            pidfController = new PIDFController(kp, kd, ki, kf);
            lift_l = hardwareMap.get(DcMotor.class, "lift_l");
            lift_r = hardwareMap.get(DcMotor.class, "lift_r");
            liftInitialized = true;

            lift_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift_r.setDirection(DcMotor.Direction.REVERSE);

            telemetry.addData("lift", "Initialized");
        } catch (IllegalArgumentException iae){
            telemetry.addData("lift", iae.getMessage());
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.update();

    }
    public void set_target_position(int position) {
        target_position = position;
    }

    public void update_pid() {
        pidfController.setPIDF(kp, kd, ki, kf);
        current_position = lift_l.getCurrentPosition();
        if(target_position == 0 && current_position < 5) {
            setpower(0);
        }
        else {
            double power = pidfController.calculate(current_position, target_position);
            setpower(power);
        }
        tel();

    }
    public void setpower(double power) {
        lift_l.setPower(power);
        lift_r.setPower(power);
    }
    public int getCurrent_position(){
        return lift_l.getCurrentPosition();
    }
    public void reset() {
        lift_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void tel() {
        telemetry.addData("lift position", getCurrent_position());
        telemetry.addData("target position", target_position);
        telemetry.update();
    }
    public void set_to_high_chamber() {
        set_target_position(high_chamber);
    }
    public void set_to_high_basket() {
        set_target_position(high_basket);
    }
    public void setpower1(double power) {
        power = -power;
        if (liftInitialized) {
            if (power > 0) {
                if (Math.abs(lift_l.getCurrentPosition()) <= 2070) {
                    lift_l.setPower(power);
                    lift_r.setPower(power);
                }
            }
            else if (power < 0) {
                if (Math.abs(lift_l.getCurrentPosition()) > 5) {
                    lift_l.setPower(power);
                    lift_r.setPower(power);
                }
            }
            else {
                if (Math.abs(lift_l.getCurrentPosition()) >= 10) {
                    hold_position();
                }
                else {
                    lift_l.setPower(0);
                    lift_r.setPower(0);
                }
            }
        }
    }

    public void up_auto() {
        if (Math.abs(lift_r.getCurrentPosition()) <= 1500) {
            lift_l.setPower(0.7);
            lift_r.setPower(0.7);
        }
        else {
            hold_position();
        }
    }
    public void up2_auto() {
        if (Math.abs(lift_r.getCurrentPosition()) >= 800) {
            lift_l.setPower(-0.2);
            lift_r.setPower(-0.2);
        }
        else {
            hold_position();
        }
    }
    public void down_auto() {
        if (Math.abs(lift_r.getCurrentPosition()) > 0) {
            lift_l.setPower(-0.2);
            lift_r.setPower(-0.2);
        }
        else {
            hold_position();
        }
    }
    public void hold_position() {
        setpower(holdp);
    }

}