package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Commands.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.TeamElementDetection.Detection;
import org.firstinspires.ftc.teamcode.hardware.Commands.MecanumBase;
@Config
@TeleOp(name="Auto_Navigation", group="Iterative")
public class AutoNavigation extends OpMode
{
    PIDController pid;
    PIDController pidlat;
    Detection detection;
    MecanumBase mecanumBase;
    GoBildaPinpointDriver odo;
    String mode = "MANUAL";
    boolean outtaketake = false, take = false, peredtake = false, borttake = false;
    public static double kp = 0, ki = 0, kd = 0;
    public static double kp2 = 0, ki2 = 0, kd2 = 0;
    public static double ax = 0;
    public static double lx = 0;
    @Override
    public void init() {
        pid = new PIDController(kp, ki, kd);
        pidlat = new PIDController(kp2, ki2, kd2);
        detection = new Detection(hardwareMap, telemetry);
        mecanumBase = new MecanumBase(hardwareMap, telemetry);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setOffsets(122.5, -146.061); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);


    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        odo.resetPosAndIMU();
    }

    @Override
    public void loop() {
        odo.update();
        if(detection.detected) {
            pid.setPID(kp, ki, kd);
            pidlat.setPID(kp2, ki2, kd2);
            double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);
            double power = pid.calculate(0, heading);
            double x = detection.getXDist();
            double y = detection.getYDist();
            double lat = pidlat.calculate(0, x);
            mecanumBase.move1(ax * y, lat, power);
            telemetry.addData("x:", x);
            telemetry.addData("h:", heading);
            telemetry.update();
        }
        else {
            robot_centric();
        }
    }


    @Override
    public void stop() {
    }


    public void robot_centric() {
        mecanumBase.move(gamepad1);
    }
}
