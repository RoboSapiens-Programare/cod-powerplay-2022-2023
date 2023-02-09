package org.firstinspires.ftc.teamcode.drive.subsystems;

import static org.firstinspires.ftc.teamcode.drive.opmode.TestEncoder.POWER;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake{
    public DcMotor motorGlisiera2;
    private Servo servoCleste, servoX, servoY;
    public double target = 0;

    public Intake(HardwareMap hardwareMap){
        PhotonCore.enable();
        motorGlisiera2 = hardwareMap.dcMotor.get("motorGlisiera2");
        servoX = hardwareMap.servo.get("servoX");
        servoY = hardwareMap.servo.get("servoY");
        servoCleste = hardwareMap.servo.get("servoCleste2");

        //Motor initialization
        motorGlisiera2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera2.setDirection(DcMotorSimple.Direction.REVERSE);

        servoX.setDirection(Servo.Direction.FORWARD);
        servoY.setDirection(Servo.Direction.FORWARD);
        servoCleste.setDirection(Servo.Direction.FORWARD);
    }

    public void strangeClesteIntake(){
        servoCleste.setPosition(0);
    }

    public void desfaClesteIntake(){
        servoCleste.setPosition(0.5);
    }

    public void desfaServoX(){
        servoX.setPosition(-1);
    }

    public void strangeServoX(){
        servoX.setPosition(1);
    }

    public void desfaServoY(){
        servoY.setPosition(-1);
    }

    public void strangeServoY(){servoX.setPosition(1);}

    public void setPowerIntake(double power){
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisiera2.setPower(power);
    }

    public void manualLevelIntake(double manualTarget) {
        motorGlisiera2.setTargetPosition((int) manualTarget);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera2.getCurrentPosition() > manualTarget)
        {
            motorGlisiera2.setPower(POWER);
        }
        else {
            motorGlisiera2.setPower(-POWER);
        }
    }

    public void setLevelIntake(int target, double multiplier){
        motorGlisiera2.setTargetPosition(-target);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera2.getCurrentPosition() > -target)
            motorGlisiera2.setPower(-POWER);
        else motorGlisiera2.setPower(POWER * multiplier);
    }
    public void setLevelIntake(int target){
        motorGlisiera2.setTargetPosition(-target);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera2.getCurrentPosition() > -target)
            motorGlisiera2.setPower(-POWER);
        else motorGlisiera2.setPower(POWER);
    }

}
