package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Robot;


@TeleOp(group = "driver")
public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    private BNO055IMU imuV2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);

        imuV2 = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuV2.initialize(parameters);

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Encoder ticks: ", robot.drive.mo)

            if(gamepad2.a) robot.sweeper.levelOne();
            if(gamepad2.x) robot.sweeper.levelTwo();
            if(gamepad2.y) robot.sweeper.levelThree();

            if(gamepad2.dpad_up) robot.sweeper.bratUp();
            if(gamepad2.dpad_down) robot.sweeper.bratDown();

            if(gamepad2.left_bumper) robot.sweeper.outtake();
            else if(gamepad2.right_bumper) robot.sweeper.intake();
            else robot.sweeper.stopSweep();

            if(gamepad2.right_stick_button) robot.sweeper.sweepIn();
            else if(gamepad2.left_stick_button) robot.sweeper.sweepOut();

            if(gamepad1.right_bumper) robot.sweeper.culcatCuva();
            if(gamepad1.x) robot.sweeper.resetCuva();

//            Carousel:
            if(gamepad1.left_bumper) robot.carousel.startCarousel();
            else if(gamepad1.dpad_up) robot.carousel.startCarouselReverse();
            else robot.carousel.stopCarousel();

//            Drive:
            robot.drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
        }
    }
}
