package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.Robot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.Glisiera;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDriveCh;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "tewt encodewr")
public class TestEncoder extends LinearOpMode {
    //    private ModernRoboticsTouchSensor touchSensor;
//    private DistanceSensor distanceSensor;
//    private ColorSensor colorSensor;
//    private ModernRoboticsI2cIrSeekerSensorV3 modernRoboticsI2cIrSeekerSensorV3;
//    private ModernRoboticsI2cGyro gyroSensor;
//    private ModernRoboticsAnalogOpticalDistanceSensor ods;
//    private ModernRoboticsI2cCompassSensor compassSensor;
//    private ModernRoboticsI2cRangeSensor range;
    private DcMotor motor1;
//    private DcMotor motor2;
    public double manualTargett = 0;
//    private CRServo servo;
//    private Encoder encoder0;
//    private Encoder encoder1;
//    private Encoder encoder2;
//    private Encoder encoder3;
//    private SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
//    private BNO055IMU imu;
    private Robot robot = null;
    public final static double POWER = 1;
    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }
    @Override
    public void runOpMode() throws InterruptedException {
//        MecanumDriveCh drive = new MecanumDriveCh(hardwareMap);
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
//        touchSensor = hardwareMap.get(ModernRoboticsTouchSensor.class, "touchSensor");
//        colorSensor = hardwareMap.get(ColorSensor.class, "coloSensor");
//        modernRoboticsI2cIrSeekerSensorV3 = hardwareMap.get(ModernRoboticsI2cIrSeekerSensorV3.class, "irSeeker");
//        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyroSensor");
//        ods = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "ods");
//        compassSensor = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass");
//        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        motor1 = hardwareMap.get(DcMotor.class, "motorGlisiera1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
//        motor2 = hardwareMap.get(DcMotor.class, "motorGlisiera1");
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
//        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        servo = hardwareMap.get(CRServo.class, "servo");
//        servo.setDirection(CRServo.Direction.FORWARD);

//        encoder0 = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
//        encoder1 = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
//        encoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
//        encoder3 = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));

//        sampleMecanumDrive.getExternalHeading();

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
//            telemetry.addData("range sensor: ", distanceSensor.getDistance(DistanceUnit.CM));
//            telemetry.addData("range sensor: ", touchSensor.getValue());
//            telemetry.addData("color", colorSensor.red());
//            telemetry.addData("Ma atingi Costica?", touchSensor.isPressed());
//            telemetry.addData("ir: ", modernRoboticsI2cIrSeekerSensorV3.signalDetected());

//            gyroSensor.calibrate();
//            telemetry.addData("gyro calibration status: ", gyroSensor.isCalibrating());
//            gyroSensor.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
//            telemetry.addData("gyro measurement mode: ", gyroSensor.getMeasurementMode());
//            telemetry.addData("gyro heading mode: ", gyroSensor.getHeadingMode());
//            telemetry.addData("status: ", gyroSensor.status());
//            telemetry.addData("gyro X: ", gyroSensor.rawX());
//            telemetry.addData("gyro Y: ", gyroSensor.rawY());
//            telemetry.addData("gyro Z: ", gyroSensor.rawZ());

//            telemetry.addData("ods light: ", ods.getLightDetected());
//            telemetry.addData("ods raw light: ", ods.getRawLightDetected());
//            telemetry.addData("ods maxvoltage: ", ods.getMaxVoltage());
//            telemetry.addData("ods raw light max: ", ods.getRawLightDetectedMax());

//            compassSensor.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
//            if (gamepad1.a) compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
//
//            telemetry.addData("compass status: ", compassSensor.status());
//            telemetry.addData("compass calibration status: ", compassSensor.isCalibrating());
//            telemetry.addData("", compassSensor.calibrationFailed());
//            telemetry.addData("compass direction: ", compassSensor.getDirection());
//            telemetry.addData("compass flux: ", compassSensor.getMagneticFlux());

//            telemetry.addData("range ultrasonic: ", range.rawUltrasonic());
//            telemetry.addData("range ultrasonic cm : ", range.cmUltrasonic());
//            telemetry.addData("range light: ", range.rawOptical());
//            telemetry.addData("range light cm: ", range.cmOptical());
//            telemetry.addData("ticksparal: ", encoderParalel.getCurrentPosition());
//            telemetry.addData("ticksperp: ", encoderPerpendicular.getCurrentPosition());

//            if(gamepad1.a) servo.setPower(-1);
//            else if(gamepad1.y) servo.setPower(1);
//            else servo.setPower(0);

//            if(gamepad2.a){
//                motor.setTargetPosition(420);
//                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                if(motor.getCurrentPosition() > 420)
//                    motor.setPower(-POWER);
//                else motor.setPower(POWER);
//            }
//
//            if(gamepad2.b){
//                motor.setTargetPosition(1500);
//                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                if(motor.getCurrentPosition() > 1500)
//                    motor.setPower(-POWER);
//                else motor.setPower(POWER);
//            }
//            if(gamepad2.circle){
//                robot.glisiera.motorGlisiera1.setPower(robot.glisiera.PIDControl(2000, robot.glisiera.motorGlisiera1.getCurrentPosition()));
//                telemetry.addData("ticks: ", robot.glisiera.motorGlisiera1.getCurrentPosition());
//            }

//            if (gamepad2.right_trigger > 0.1) {
//                robot.glisiera.manualTarget = robot.glisiera.motorGlisiera1.getCurrentPosition() + calculateThrottle(gamepad2.right_trigger * 15);
//                robot.glisiera.manualLevel(robot.glisiera.manualTarget);
//            }
//            if (gamepad2.left_trigger > 0.1) {
//                robot.glisiera.manualTarget = robot.glisiera.motorGlisiera1.getCurrentPosition() - calculateThrottle(gamepad2.left_trigger * 15);
//                robot.glisiera.manualTarget--;
//                robot.glisiera.manualLevel(robot.glisiera.manualTarget);
//            }
//
//
//            if(gamepad2.y){
//                motor.setTargetPosition(2200);
//                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                if(motor.getCurrentPosition() > 2200)
//                    motor.setPower(-POWER);
//                else motor.setPower(POWER);
//            }
            if(gamepad2.left_trigger > 0.1){
                motor1.setPower(1);
            }
            if(gamepad2.right_trigger > 0.1){
                motor1.setPower(-1);
            }
//


            telemetry.addData("ticks1: ", motor1.getCurrentPosition());

//            telemetry.addData("imu: ", imu.isGyroCalibrated());
//            telemetry.addData("imu: ", imu.getCalibrationStatus());
//            telemetry.addData("imu: ", imu.getPosition());
//            telemetry.addData("imu: ", imu.getSystemStatus());
//            telemetry.addData("imu: ", imu.getAngularOrientation());

//            telemetry.addData("Encoder ticks: ", encoder0.getCurrentPosition());
//            telemetry.addData("Encoder ticks: ", encoder1.getCurrentPosition());
//            telemetry.addData("Encoder ticks: ", encoder2.getCurrentPosition());
//            telemetry.addData("Encoder ticks: ", encoder3.getCurrentPosition());
            telemetry.update();

        }
    }

}