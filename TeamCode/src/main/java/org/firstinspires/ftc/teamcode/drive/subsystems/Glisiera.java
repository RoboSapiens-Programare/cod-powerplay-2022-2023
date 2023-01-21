package org.firstinspires.ftc.teamcode.drive.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.opmode.TestEncoder.POWER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.function.Pow;

public class Glisiera {
    public DcMotor motorGlisiera1;
    private Servo cleste;
    public double manualTarget = 0;


    public Glisiera(HardwareMap hardwareMap){
        motorGlisiera1 = hardwareMap.dcMotor.get("motorGlisiera1");
        cleste = hardwareMap.servo.get("servoCleste");

        //Motor initialization
        motorGlisiera1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera1.setDirection(DcMotorSimple.Direction.FORWARD);

        cleste.setDirection(Servo.Direction.REVERSE);
    }

    public void strangeCleste(){
        cleste.setPosition(-0.5);
    }

    public void desfaCleste(){
        cleste.setPosition(0.5);
    }

    public void setPower(double power){
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisiera1.setPower(power);
    }

    public void groundLevel(){
        motorGlisiera1.setTargetPosition(420);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera1.getCurrentPosition() > 420) {
            motorGlisiera1.setPower(-POWER);
        }
        else {
            motorGlisiera1.setPower(POWER);
        }
        }

    public void lowLevel(){
        motorGlisiera1.setTargetPosition(1500);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera1.getCurrentPosition() > 1500)
            motorGlisiera1.setPower(-POWER);
        else motorGlisiera1.setPower(POWER);
    }
    public void mediumLevel(){
        motorGlisiera1.setTargetPosition(2400);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera1.getCurrentPosition() > 2400)
            motorGlisiera1.setPower(-POWER);
        else motorGlisiera1.setPower(POWER);
    }
    public void tallLevel() {
        motorGlisiera1.setTargetPosition(3300);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera1.getCurrentPosition() > 3300)
            motorGlisiera1.setPower(-POWER);
        else motorGlisiera1.setPower(POWER);
    }

    public void zeroLevel(){
        motorGlisiera1.setTargetPosition(50);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera1.getCurrentPosition() > 50)
            motorGlisiera1.setPower(-POWER);
        else motorGlisiera1.setPower(POWER);
    }

    public void manualLevel(double manualTarget) {
        motorGlisiera1.setTargetPosition((int) manualTarget);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera1.getCurrentPosition() > manualTarget)
        {
            motorGlisiera1.setPower(-POWER);
        }
        else {
            motorGlisiera1.setPower(POWER);
        }
    }

}


