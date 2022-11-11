package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.subsystems.Odometrie;


@TeleOp(group = "driver")
public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    DcMotorEx odo;
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        odo = hardwareMap.get(DcMotorEx.class, "encoder");
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

//            if(gamepad1.right_trigger > 0.1) robot.glisiera.setPower(gamepad1.right_trigger);
//            else if(gamepad1.left_trigger > 0.1) robot.glisiera.setPower(-gamepad1.left_trigger);
//            else robot.glisiera.setPower(0);

//            Drive:
            robot.drive.setDrivePower(new Pose2d(gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2, gamepad1.right_stick_x / 2));
            telemetry.addData("Encoder value", (float)odo.getCurrentPosition() / 8192.0f * Math.PI * 5 + "cm");
            telemetry.update();
            }
        }
    }