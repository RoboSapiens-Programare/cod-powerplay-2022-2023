package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GlisieraDemo {
    private DcMotor motorGlisiera1;
    private DcMotor motorGlisiera2;
    private Servo cleste;

    private static final int GROUND = 0, LOW = 1, MEDIUM = 2, TALL = 3;

    public GlisieraDemo(HardwareMap hardwareMap){
        motorGlisiera1 = hardwareMap.dcMotor.get("motorGlisiera1");
//        motorGlisiera2 = hardwareMap.dcMotor.get("motorGlisiera2");
        cleste = hardwareMap.servo.get("servoCleste");

        //Motor initialization
        motorGlisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorGlisiera1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        motorGlisiera2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorGlisiera2.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorGlisiera2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cleste.setDirection(Servo.Direction.REVERSE);
    }

    public void setPower(double power){
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisiera1.setPower(power);
    }

    public void strangeCleste(){
        cleste.setPosition(-0.5);
    }

    public void desfaCleste(){
        cleste.setPosition(0.5);
    }

    public void groundLevel(){
        motorGlisiera1.setTargetPosition(GROUND);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorGlisiera1.setPower(0.75);
    }
    public void lowLevel(){
        motorGlisiera1.setTargetPosition(LOW);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorGlisiera1.setPower(0.75);
    }
    public void mediumLevel(){
        motorGlisiera1.setTargetPosition(MEDIUM);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorGlisiera1.setPower(0.75);
    }
    public void tallLevel(){
        motorGlisiera1.setTargetPosition(TALL);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorGlisiera1.setPower(0.75);
    }
}
