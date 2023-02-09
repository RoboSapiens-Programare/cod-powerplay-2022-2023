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
    public final static double DOWN_MULTIPLIER = 0.2;
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
                robot.outtake.strangeCleste();
                gamepad1.rumble(500);
            }
            if (gamepad2.left_bumper) robot.outtake.desfaCleste();

            if (gamepad2.dpad_left) robot.outtake.setPower(0);
            if (gamepad2.dpad_up) {
                robot.intake.setLevelIntake(TALL);
                sleep(2500);
                robot.intake.strangeServoX();
                sleep(500);
                robot.intake.strangeClesteIntake();
                sleep(500);
                robot.intake.strangeServoY();
                robot.intake.setLevelIntake(ZERO);
                sleep(500);
                robot.intake.desfaClesteIntake();
                robot.outtake.strangeCleste();
            }
            if(gamepad1.dpad_left){
                robot.intake.strangeClesteIntake();
            }
            if(gamepad1.dpad_right){
                robot.intake.strangeServoY();
            }if(gamepad1.dpad_up){
                robot.intake.strangeServoX();
            }if(gamepad1.dpad_down){
                robot.intake.desfaServoX();
            }
            if(gamepad1.a){
                robot.intake.desfaClesteIntake();
            }if(gamepad1.b){
                robot.intake.desfaServoY();
            }
            //Aimbot
            if (gamepad2.touchpad) robot.outtake.setLevel(ZERO, DOWN_MULTIPLIER);
            if (gamepad2.cross) robot.outtake.setLevel(GROUND, DOWN_MULTIPLIER);
            if (gamepad2.circle) robot.outtake.setLevel(LOW, DOWN_MULTIPLIER);
            if (gamepad2.triangle) robot.outtake.setLevel(MEDIUM, DOWN_MULTIPLIER);
            if (gamepad2.square) robot.outtake.setLevel(TALL, DOWN_MULTIPLIER);

            //Manual
            if (gamepad2.left_trigger > 0.1) {
                robot.outtake.manualTarget = robot.outtake.motorGlisiera1.getCurrentPosition() + calculateThrottle(gamepad2.left_trigger * 12);
                robot.outtake.manualLevel(robot.outtake.manualTarget);
            }
            if (gamepad2.right_trigger > 0.1) {
                robot.outtake.manualTarget = robot.outtake.motorGlisiera1.getCurrentPosition() - calculateThrottle(gamepad2.right_trigger * 12);
                robot.outtake.manualTarget--;
                robot.outtake.manualLevel(robot.outtake.manualTarget);
            }

            if (gamepad1.left_trigger > 0.1) {
                robot.intake.target = robot.intake.motorGlisiera2.getCurrentPosition() + calculateThrottle(gamepad1.left_trigger * 12);
                robot.intake.manualLevelIntake(robot.intake.target);
            }
            if (gamepad1.right_trigger > 0.1) {
                robot.intake.target = robot.intake.motorGlisiera2.getCurrentPosition() - calculateThrottle(gamepad1.right_trigger * 12);
                robot.intake.target--;
                robot.intake.manualLevelIntake(robot.intake.target);
            }
                //Drive
                robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)), calculateThrottle((float) (-gamepad1.left_stick_x)), calculateThrottle((float) (-gamepad1.right_stick_x))));

//            telemetry.addData("Encoder value", (float)odo.getCurrentPosition() / 8192.0f * Math.PI * 5 + "cm");
//            Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
                telemetry.addData("servo(outtake): ", robot.outtake.cleste.getPosition());
                telemetry.addData("Motor1glisiera: ", robot.outtake.motorGlisiera1.getCurrentPosition());
                telemetry.addData("Motor2glisiera: ", robot.intake.motorGlisiera2.getCurrentPosition());
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