package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.Robot;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Odometrie;

@TeleOp(group = "driver")
public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private double reference = 1500;
    public final static int ZERO = 0, GROUND = 100, LOW = 900, MEDIUM = 1550, TALL = 2100;
    public final static double DOWN_MULTIPLIER = 0.5;
    private int direction = 0;

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
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            if (gamepad2.right_bumper) {
                robot.glisiera.strangeCleste();
                gamepad1.rumble(500);
            }
            if (gamepad2.left_bumper) robot.glisiera.desfaCleste();

            if (gamepad2.dpad_left) robot.glisiera.setPower(0);

            //Aimbot
            if (gamepad2.touchpad) robot.glisiera.setLevel(ZERO, DOWN_MULTIPLIER);
            if (gamepad2.cross) robot.glisiera.setLevel(GROUND, DOWN_MULTIPLIER);
            if (gamepad2.circle) robot.glisiera.setLevel(LOW, DOWN_MULTIPLIER);
            if (gamepad2.triangle) robot.glisiera.setLevel(MEDIUM, DOWN_MULTIPLIER);
            if (gamepad2.square) robot.glisiera.setLevel(TALL, DOWN_MULTIPLIER);

            //Manual
            if (gamepad2.left_trigger > 0.1) {
                robot.glisiera.manualTarget = robot.glisiera.motorGlisiera1.getCurrentPosition() + calculateThrottle(gamepad2.left_trigger * 12);
                robot.glisiera.manualLevel(robot.glisiera.manualTarget);
            }
            if (gamepad2.right_trigger > 0.1) {
                robot.glisiera.manualTarget = robot.glisiera.motorGlisiera1.getCurrentPosition() - calculateThrottle(gamepad2.right_trigger * 12);
                robot.glisiera.manualTarget--;
                robot.glisiera.manualLevel(robot.glisiera.manualTarget);
            }
                //Drive
                robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)), calculateThrottle((float) (-gamepad1.left_stick_x)), calculateThrottle((float) (-gamepad1.right_stick_x))));

//            telemetry.addData("Encoder value", (float)odo.getCurrentPosition() / 8192.0f * Math.PI * 5 + "cm");
//            Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
                telemetry.addData("servo", robot.glisiera.cleste.getPosition());
                telemetry.addData("Motor1glisiera", robot.glisiera.motorGlisiera1.getCurrentPosition());
//            telemetry.addData("Left front motor current", robot.drive.leftFront.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Left rear motor current", robot.drive.leftRear.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Right front motor current", robot.drive.rightFront.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Right rear motor current", robot.drive.rightRear.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Slider ticks", robot.glisiera.motorGlisiera1.getCurrentPosition());
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