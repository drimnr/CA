package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Commands.Horizontal_Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.Intake;
import org.firstinspires.ftc.teamcode.hardware.Commands.Lift;
import org.firstinspires.ftc.teamcode.hardware.Commands.MecanumBase;
import org.firstinspires.ftc.teamcode.hardware.Commands.Outtake;

@TeleOp(name="Sdu linear teleop", group="Linear")

public class SDU_TeleOp extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timerbt = new ElapsedTime();
    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timerpd = new ElapsedTime();
    Lift lift;
    Intake intake;
    Outtake outtake;
    Horizontal_Lift horlift;
    MecanumBase mecanumBase;
    String mode = "MANUAL";
    boolean outtaketake = false, take = false, peredtake = false, borttake = false;
    @Override
    public void runOpMode() {
        lift = new Lift(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        horlift = new Horizontal_Lift(hardwareMap, telemetry);
        mecanumBase = new MecanumBase(hardwareMap, telemetry);



        telemetry.addData("Status", "Initialized");
        //telemetry.addData("Right Stick Y", gamepad2.right_stick_y);
        //telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.update();
        timer.reset();
        outtake.mayat_down();
        outtake.release();
        waitForStart();


        timer.reset();
        timerbt.reset();
        timer1.reset();
        timerpd.reset();


        while (opModeIsActive()) {
            // intake
            if (gamepad1.dpad_up) {
                intake.rotate_mid();
            }
            if (gamepad1.dpad_left) {
                intake.rotate_left();
            }
            if (gamepad1.dpad_right) {
                intake.rotate_right();
            }

            if (gamepad2.a || gamepad1.right_bumper) {
                take = true;
                timer1.reset();
                intake.setsample_take();
                intake.open();
            }
            if (take && timer1.milliseconds() > 100) {
                intake.close();
            }
            if (take && timer1.milliseconds() > 500) {
                intake.setmidpovishe_take();
                take = false;
            }
            if (gamepad2.b) {
                intake.setmid_take();
                intake.open();
            }
            if (gamepad2.x) {
                timerpd.reset();
                peredtake = true;
                intake.open_chut();
            }
            if(peredtake && timerpd.milliseconds() > 200){
                intake.rotate_mid();
                intake.setperedacha();
            }
            if(peredtake && timerpd.milliseconds() > 500){
                intake.close();
                peredtake = false;
            }


            if (gamepad2.right_bumper) {
                horlift.open();
                intake.setmidpovishe_take();
                intake.open();
            }
            //horizontal lift
            if (gamepad2.left_bumper) {
                outtake.setPered_take();
                horlift.close();
            }



            //outtake
            if (gamepad2.dpad_up) {
                outtaketake = true;
                outtake.grab();
                timer.reset();
            }
            if (outtaketake && timer.milliseconds() > 300) {
                intake.setmid_take();
                intake.open();
                outtake.mayat_mid();outtake.sethb();outtaketake = false;
            }
            if (gamepad1.a || gamepad2.dpad_down) {
                outtake.release();
                outtake.mayat_down();
                outtake.setZad_take();
            }
            if (gamepad1.y || gamepad2.dpad_left) {
                timerbt.reset();
                borttake = true;
                outtake.grab();
            }
            if (borttake && timerbt.milliseconds() > 250) {
                outtake.mayat_up();
                outtake.setspecimen();
                borttake = false;
            }

            if (gamepad2.y || gamepad1.left_bumper) {
                outtake.release();
            }






















            //drivetrain
            if (gamepad1.dpad_down) {
                reset();
            }

            robot_centric();

            //lift
            if (mode == "PID") {
                if (gamepad2.right_stick_y != 0) {
                    mode = "MANUAL";
                }
                lift.update_pid();
            }
            else {
                if (gamepad2.left_stick_button) {
                    mode = "PID";
                    lift.set_to_high_chamber();
                }
                lift.setpower1(gamepad2.right_stick_y);
            }








        }
    }








    public void reset() {

    }
    public void field_centric() {

    }
    public void robot_centric() {
        mecanumBase.move(gamepad1);
    }
}