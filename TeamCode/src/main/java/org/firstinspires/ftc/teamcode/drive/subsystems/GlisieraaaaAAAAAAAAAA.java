package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.TestEncoder;

public class GlisieraaaaAAAAAAAAAA {
    public DcMotor motorGlisiera1;
    private DcMotor motorGlisiera2;
    private Servo cleste;

    private double integralSum = 0;
    private double kp = 1;
    private double ki = 0;
    private double kd = 0;
    private double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    private static final int ZERO = 69, GROUND = 420, LOW = 1500, MEDIUM = 2200, TALL = 3300;

    public GlisieraaaaAAAAAAAAAA(HardwareMap hardwareMap){
        motorGlisiera1 = hardwareMap.dcMotor.get("motorGlisiera1");
//        motorGlisiera2 = hardwareMap.dcMotor.get("motorGlisiera2");
        cleste = hardwareMap.servo.get("servoCleste");

        // TODO Motor initialization
        motorGlisiera1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera1.setDirection(DcMotorSimple.Direction.FORWARD);

//        motorGlisiera2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorGlisiera2.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorGlisiera2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cleste.setDirection(Servo.Direction.FORWARD);
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

    public void zeroLevel(){
        motorGlisiera1.setTargetPosition(0);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorGlisiera1.setPower(-TestEncoder.POWER);
    }

    public void groundLevel(){
        motorGlisiera1.setTargetPosition(420);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera1.getCurrentPosition() > GROUND)
            motorGlisiera1.setPower(-TestEncoder.POWER);
        else motorGlisiera1.setPower(TestEncoder.POWER);
    }
    public void lowLevel(){
        motorGlisiera1.setTargetPosition(1500);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motorGlisiera1.getCurrentPosition() > LOW)
            motorGlisiera1.setPower(-TestEncoder.POWER);
        else motorGlisiera1.setPower(TestEncoder.POWER);
    }
    public void mediumLevel(){
        motorGlisiera1.setTargetPosition(2200);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motorGlisiera1.getCurrentPosition() > MEDIUM)
            motorGlisiera1.setPower(-TestEncoder.POWER);
        else motorGlisiera1.setPower(TestEncoder.POWER);
    }
    public void tallLevel(){
        motorGlisiera1.setTargetPosition(TALL);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorGlisiera1.setPower(TestEncoder.POWER);
    }

    public double PIDControl(double reference, double state){
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * kp) + (derivative * kd) + (integralSum * ki);
        return output;
    }



    public void setPowerPid(int target)
    {
        if(target > motorGlisiera1.getCurrentPosition()){
            motorGlisiera1.setPower(PIDControl(target, motorGlisiera1.getCurrentPosition()));
        }

        else{
            motorGlisiera1.setPower(-PIDControl(target, motorGlisiera1.getCurrentPosition()));
        }
    }

}
