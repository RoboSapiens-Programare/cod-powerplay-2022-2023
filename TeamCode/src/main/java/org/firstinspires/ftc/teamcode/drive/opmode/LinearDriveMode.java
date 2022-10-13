package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;


@TeleOp(group = "driver")
public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
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

//            Drive:
            robot.drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
        }
    }
}
