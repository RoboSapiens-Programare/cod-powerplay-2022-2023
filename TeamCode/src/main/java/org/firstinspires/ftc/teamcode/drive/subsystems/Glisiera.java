package org.firstinspires.ftc.teamcode.drive.subsystems;

import static com.sun.tools.doclint.Entity.and;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.opmode.TestEncoder.POWER;

import android.media.session.PlaybackState;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Pow;

public class Glisiera {
    public DcMotorEx motorGlisiera1;
    public DcMotorEx motorGlisiera2;
    private Servo cleste;
    public double manualTarget = 0;

    private double integralSum = 0;
    private double Kp = 0.43;
    private double Ki = 0;
    private double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastErorr = 0;


    public Glisiera(HardwareMap hardwareMap){
        motorGlisiera1 = hardwareMap.get(DcMotorEx.class, "motorGlisiera1");
        motorGlisiera2 = hardwareMap.get(DcMotorEx.class, "motorGlisiera2");
        cleste = hardwareMap.servo.get("servoCleste");

        //Motor initialization
        motorGlisiera1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisiera1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorGlisiera2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisiera2.setDirection(DcMotorSimple.Direction.FORWARD);

        cleste.setDirection(Servo.Direction.REVERSE);
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastErorr) / timer.seconds();

        lastErorr = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public void strangeCleste(){
        cleste.setPosition(-0.5);
    }

    public void desfaCleste(){
        cleste.setPosition(0.5);
    }



    public void groundLevel(){
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(motorGlisiera1.getCurrentPosition() > 420) {
//            motorGlisiera1.setPower(-power);
//            motorGlisiera2.setPower(power);
        }
        else {
//            motorGlisiera1.setPower(power);
//            motorGlisiera2.setPower(-power);
        }
        }

    public void lowLevel(){
        motorGlisiera1.setTargetPosition(-1500);
//        motorGlisiera2.setTargetPosition(1500);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motorGlisiera1.getCurrentPosition() <= 1420){
            motorGlisiera1.setPower(-1);
            motorGlisiera2.setPower(1);
//            if(motorGlisiera1.getCurrentPosition() < -1600) {
//                motorGlisiera1.setPower(-POWER);
//                motorGlisiera2.setPower(motorGlisiera1.getPower());
//            }
//            else {
//                motorGlisiera1.setPower(POWER);
//                motorGlisiera2.setPower(motorGlisiera1.getPower());
//            }
        }

        while(motorGlisiera1.getCurrentPosition() >= 1490){
            motorGlisiera1.setPower(1);
            motorGlisiera2.setPower(-1);
        }

        if (motorGlisiera1.getCurrentPosition() > 1420 && motorGlisiera1.getCurrentPosition() < 1490) {
            motorGlisiera2.setPower(0);
            motorGlisiera1.setPower(0);
        }


    }
    public void mediumLevel(){
        motorGlisiera1.setTargetPosition(-2400);
//        motorGlisiera2.setTargetPosition(1500);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motorGlisiera1.getCurrentPosition() <= 2350){
            motorGlisiera1.setPower(-1);
            motorGlisiera2.setPower(1);
//            if(motorGlisiera1.getCurrentPosition() < -1600) {
//                motorGlisiera1.setPower(-POWER);
//                motorGlisiera2.setPower(motorGlisiera1.getPower());
//            }
//            else {
//                motorGlisiera1.setPower(POWER);
//                motorGlisiera2.setPower(motorGlisiera1.getPower());
//            }
        }

        while(motorGlisiera1.getCurrentPosition() >= 2420) {
            motorGlisiera1.setPower(1);
            motorGlisiera2.setPower(-1);
        }

        if (motorGlisiera1.getCurrentPosition() > 2350 && motorGlisiera1.getCurrentPosition() < 2420) {
            motorGlisiera2.setPower(0);
            motorGlisiera1.setPower(0);
        }
    }
    public void tallLevel() {
//        motorGlisiera1.setTargetPosition(-3300);
////        motorGlisiera2.setTargetPosition(1500);
//        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motorGlisiera1.getCurrentPosition() <= 3300){
            motorGlisiera2.setPower(1);
            motorGlisiera1.setPower(-1);

//            if(motorGlisiera1.getCurrentPosition() < -1600) {
//                motorGlisiera1.setPower(-POWER);
//                motorGlisiera2.setPower(motorGlisiera1.getPower());
//            }
//            else {
//                motorGlisiera1.setPower(POWER);
//                motorGlisiera2.setPower(motorGlisiera1.getPower());
//            }
        }

        while(motorGlisiera1.getCurrentPosition() >= 3370) {
            motorGlisiera2.setPower(-1);
            motorGlisiera1.setPower(1);
        }

        if (motorGlisiera1.getCurrentPosition() > 3300 && motorGlisiera1.getCurrentPosition() < 3370) {
            motorGlisiera2.setPower(0);
            motorGlisiera1.setPower(0);
        }
    }

    public void zeroLevel(){
        motorGlisiera1.setTargetPosition(-50);
//        motorGlisiera2.setTargetPosition(1500);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motorGlisiera1.getCurrentPosition() <= 40){
            motorGlisiera1.setPower(-1);
            motorGlisiera2.setPower(1);
//            if(motorGlisiera1.getCurrentPosition() < -1600) {
//                motorGlisiera1.setPower(-POWER);
//                motorGlisiera2.setPower(motorGlisiera1.getPower());
//            }
//            else {
//                motorGlisiera1.setPower(POWER);
//                motorGlisiera2.setPower(motorGlisiera1.getPower());
//            }
        }

        while(motorGlisiera1.getCurrentPosition() >= 100) {
            motorGlisiera1.setPower(1);
            motorGlisiera2.setPower(-1);
        }

        if (motorGlisiera1.getCurrentPosition() > 40 && motorGlisiera1.getCurrentPosition() < 100) {
            motorGlisiera2.setPower(0);
            motorGlisiera1.setPower(0);
        }
    }

    public void manualLevel(double manualTarget) {
        motorGlisiera1.setTargetPosition((int) manualTarget);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(motorGlisiera1.getCurrentPosition() <= 20){
            motorGlisiera1.setPower(1);
            motorGlisiera2.setPower(1);
        }
        motorGlisiera2.setPower(0);
        motorGlisiera1.setPower(0);
        motorGlisiera1.setPower(PIDControl(-manualTarget, motorGlisiera1.getCurrentPosition()));
    }


}


