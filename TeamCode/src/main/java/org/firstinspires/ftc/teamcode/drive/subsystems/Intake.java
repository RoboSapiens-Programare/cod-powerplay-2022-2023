package org.firstinspires.ftc.teamcode.drive.subsystems;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.drive.opmode.TestEncoder.POWER;
import static org.firstinspires.ftc.teamcode.drive.opmode.TestEncoder.somn;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake{
    public DcMotor motorGlisiera2;
    public Servo servoCleste2, servoX, servoY;
    public double target = 0;
    private final int TICKS_PER_CENTIMETER = 22;
    public DistanceSensor senzorIntake;
    public int targetSenzor;
    private int it=0;

    public Intake(HardwareMap hardwareMap){
        PhotonCore.enable();
        motorGlisiera2 = hardwareMap.dcMotor.get("motorGlisiera2");
        servoX = hardwareMap.servo.get("servoX");
        servoY = hardwareMap.servo.get("servoY");
        servoCleste2 = hardwareMap.servo.get("servoCleste2");
        senzorIntake = hardwareMap.get(DistanceSensor.class, "senzorIntake");

        //Motor initialization
        motorGlisiera2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera2.setDirection(DcMotorSimple.Direction.REVERSE);

        servoX.setDirection(Servo.Direction.FORWARD);
        servoY.setDirection(Servo.Direction.FORWARD);
        servoCleste2.setDirection(Servo.Direction.FORWARD);
    }

    public void strangeClesteIntake(){
        servoCleste2.setPosition(0);
    }

    public void desfaClesteIntake(){
        servoCleste2.setPosition(0.5);
    }

    public void invarteClesteStanga(){
        servoX.setPosition(1);
    }

    public void invarteClesteDreapta(){
        servoX.setPosition(0);
    }

    public void ridicaCleste(){
        servoY.setPosition(1);
    }

    public void coboaraCleste(){
        servoY.setPosition(0);
    }

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

    public void autoExtend(){
         targetSenzor = ((((int)senzorIntake.getDistance(DistanceUnit.CM))) + 7) * TICKS_PER_CENTIMETER;
    motorGlisiera2.setTargetPosition(targetSenzor);
    motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera2.getCurrentPosition() < targetSenzor)
            motorGlisiera2.setPower(-POWER);
        else motorGlisiera2.setPower(POWER);

    }

    public void setSlidePosition(int target){
        motorGlisiera2.setTargetPosition(target);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera2.getCurrentPosition() < target)
            motorGlisiera2.setPower(-POWER);
        else motorGlisiera2.setPower(POWER);
    }

    public void setSlidePosition(int target, double multiplier){
        motorGlisiera2.setTargetPosition(target);
        motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera2.getCurrentPosition() < target)
            motorGlisiera2.setPower(-POWER);
        else motorGlisiera2.setPower(POWER * multiplier);
    }
    public boolean isGoing(int target){
        if (motorGlisiera2.getCurrentPosition() < target + 10 && motorGlisiera2.getCurrentPosition() > target - 10){
            return  false;
        }
        return true;

    }

    public void firstIntakeSequence(){
        autoExtend();
        servoX.setPosition(0);
        servoCleste2.setPosition(0.5);
        servoY.setPosition(0.19);
        somn(250);
        while(isGoing(targetSenzor)){}
        somn(100);
        servoCleste2.setPosition(0);
    }

    public void secondIntakeSequence() {
        servoY.setPosition(0.4);
        somn(100);
        servoX.setPosition(1);
        somn(100);
        servoY.setPosition(0.7);
        somn(500);
        setSlidePosition(750);
        while(isGoing(750));
        somn(100);
        servoY.setPosition(0.92);
        somn(200);
        servoCleste2.setPosition(0.3);
        somn(400);
        servoY.setPosition(0.7);
        somn(200);
        servoCleste2.setPosition(0);
        somn(100);
        servoY.setPosition(0.4);
        somn(400);
        servoX.setPosition(0);

    }

}
