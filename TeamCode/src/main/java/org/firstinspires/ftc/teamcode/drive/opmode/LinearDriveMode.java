package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import android.graphics.Color;
import android.os.Build;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.Robot;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Odometrie;

@TeleOp(group = "driver")
public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;

    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }

//    private int calculateSliderMotor(int x) {
//        if (robot.glisieraDemo.motorGlisiera1.getCurrentPosition() >= x)
//            return (x - robot.glisieraDemo.motorGlisiera1.getCurrentPosition());
//        else return -(x - robot.glisieraDemo.motorGlisiera1.getCurrentPosition());
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);

        gamepad1.setLedColor(200, 0, 200, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(255, 125, 0, Gamepad.LED_DURATION_CONTINUOUS);


        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad2.left_bumper) robot.glisiera.strangeCleste();
            if(gamepad2.right_bumper) robot.glisiera.desfaCleste();

            if (gamepad2.cross) robot.glisiera.groundLevel();
            if (gamepad2.circle) robot.glisiera.lowLevel();
            if (gamepad2.triangle) robot.glisiera.mediumLevel();
            if (gamepad2.square) robot.glisiera.tallLevel();
            if (gamepad2.touchpad) robot.glisiera.zeroLevel();
            if (gamepad2.right_trigger > 0.1) {
                robot.glisiera.manualTarget = robot.glisiera.motorGlisiera1.getCurrentPosition() + calculateThrottle(gamepad2.right_trigger * 15);
                robot.glisiera.manualLevel(robot.glisiera.manualTarget);
            }
            if (gamepad2.left_trigger > 0.1) {
                robot.glisiera.manualTarget = robot.glisiera.motorGlisiera1.getCurrentPosition() - calculateThrottle(gamepad2.left_trigger * 15);
                robot.glisiera.manualTarget--;
                robot.glisiera.manualLevel(robot.glisiera.manualTarget);
            }
            telemetry.addData("right trigger", gamepad2.right_trigger);
//            Drive:

//            robot.drive.setDrivePower(new Pose2d((Math.pow((100.0*(((double)gamepad1.left_stick_y/ 100.0))), 3.3219)), (Math.pow((100.0*(((double)gamepad1.left_stick_x/ 100.0))), 3.3219)), (Math.pow((100.0*(((double)gamepad1.right_stick_x/ 100.0))), 3.3219))));
            //robot.drive.setDrivePower(new Pose2d(calculateThrottle(gamepad1.left_stick_y / 2), calculateThrottle(gamepad1.left_stick_x / 2), calculateThrottle(gamepad1.right_stick_x / 2)));
            // 100*(I/100)^3.3219
            robot.drive.setDrivePower(new Pose2d(calculateThrottle(-gamepad1.left_stick_y / 2), calculateThrottle(gamepad1.left_stick_x / 2), calculateThrottle(-gamepad1.right_stick_x / 2)));
//            telemetry.addData("Encoder value", (float)odo.getCurrentPosition() / 8192.0f * Math.PI * 5 + "cm");

            telemetry.addData("Slider ticks", robot.glisiera.motorGlisiera1.getCurrentPosition());
//            telemetry.addData("left stick y", calculateThrottle(gamepad1.left_stick_y));
//            telemetry.addData("left stick y real", gamepad1.left_stick_y);
//            telemetry.addData("left stick x", calculateThrottle(gamepad1.left_stick_x));
//            telemetry.addData("left stick x real", gamepad1.left_stick_x);
//            telemetry.addData("stick stick x", calculateThrottle(gamepad1.right_stick_x));
//            telemetry.addData("right stick x real", gamepad1.right_stick_x);

            telemetry.update();
            }
        }
    }