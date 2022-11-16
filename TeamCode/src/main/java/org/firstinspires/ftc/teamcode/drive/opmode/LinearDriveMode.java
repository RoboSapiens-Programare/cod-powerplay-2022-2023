package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import android.app.backup.BackupDataOutput;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.Robot;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Odometrie;

@TeleOp(group = "driver")
public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    DcMotorEx odo;
//    public double power(double x, int p) {
//        double prod = 1;
//        for(int i = 0; i < p; i++) {
//            prod *= x;
//        }
//        return prod;
//    }

    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100*(abs(x)/100), 2);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
//        odo = hardwareMap.get(DcMotorEx.class, "encoder");
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.right_trigger > 0.1) robot.glisiera.setPower(gamepad1.right_trigger);
            else if(gamepad1.left_trigger > 0.1) robot.glisiera.setPower(-gamepad1.left_trigger);
            else robot.glisiera.setPower(0);

            if(gamepad1.a) robot.glisiera.strangeCleste();
            if(gamepad1.y) robot.glisiera.desfaCleste();

//            Drive:

//            robot.drive.setDrivePower(new Pose2d((Math.pow((100.0*(((double)gamepad1.left_stick_y/ 100.0))), 3.3219)), (Math.pow((100.0*(((double)gamepad1.left_stick_x/ 100.0))), 3.3219)), (Math.pow((100.0*(((double)gamepad1.right_stick_x/ 100.0))), 3.3219))));
            robot.drive.setDrivePower(new Pose2d(calculateThrottle(gamepad1.left_stick_y / 2), calculateThrottle(gamepad1.left_stick_x / 2), calculateThrottle(gamepad1.right_stick_x / 2)));
            // 100*(I/100)^3.3219
//            telemetry.addData("Encoder value", (float)odo.getCurrentPosition() / 8192.0f * Math.PI * 5 + "cm");
            telemetry.addData("left stick y", calculateThrottle(gamepad1.left_stick_y));
            telemetry.addData("left stick y real", gamepad1.left_stick_y);
            telemetry.addData("left stick x", calculateThrottle(gamepad1.left_stick_x));
            telemetry.addData("left stick x real", gamepad1.left_stick_x);
            telemetry.addData("stick stick x", calculateThrottle(gamepad1.right_stick_x));
            telemetry.addData("right stick x real", gamepad1.right_stick_x);

            telemetry.update();
            }
        }
    }