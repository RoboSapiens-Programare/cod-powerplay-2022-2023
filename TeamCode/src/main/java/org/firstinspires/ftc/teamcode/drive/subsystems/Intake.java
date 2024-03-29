package org.firstinspires.ftc.teamcode.drive.subsystems;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.opmode.TestEncoder.POWER;
import static org.firstinspires.ftc.teamcode.drive.opmode.TestEncoder.POWERRR;
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
//    public DistanceSensor senzorIntake;
    public DistanceSensor senzorCleste;
    public int targetSenzor;
    private int it = 0;
    public double manualTarget = 0;
    //mai trebuie 40 de tickuri
    private final int STACK_TICKS = 1540;
    private final int STACK_TICKS2 = 1735;

    public Intake(HardwareMap hardwareMap){
        PhotonCore.enable();
        motorGlisiera2 = hardwareMap.dcMotor.get("motorGlisiera2");
        servoX = hardwareMap.servo.get("servoX");
        servoY = hardwareMap.servo.get("servoY");
        servoCleste2 = hardwareMap.servo.get("servoCleste2");
//        senzorIntake = hardwareMap.get(DistanceSensor.class, "senzorIntake");
        senzorCleste = hardwareMap.get(DistanceSensor.class, "senzorCleste");
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
    //todo bag pl in marinas
    public void AutonomousExtend(){
            motorGlisiera2.setTargetPosition(STACK_TICKS);
            motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(motorGlisiera2.getCurrentPosition() < STACK_TICKS)
                motorGlisiera2.setPower(-POWER);
            else motorGlisiera2.setPower(POWER);
        }


    public void autoExtend(int targetSenzor){
        if(senzorCleste.getDistance(DistanceUnit.CM) > 6){
    motorGlisiera2.setTargetPosition(targetSenzor++ * 4);
    motorGlisiera2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera2.getCurrentPosition() < targetSenzor++ * 4)
            motorGlisiera2.setPower(-POWER);
        else motorGlisiera2.setPower(POWER);
            }
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
        if (motorGlisiera2.getCurrentPosition() < target + 30 && motorGlisiera2.getCurrentPosition() > target - 30){
            return  false;
        }
        return true;

    }

    public boolean isGoingAutonomous(int target){
        if (motorGlisiera2.getCurrentPosition() < target + 40 && motorGlisiera2.getCurrentPosition() > target - 40){
            return  false;
        }
        return true;

    }

    public void firstAutonomousIntakeSequence(){
        setSlidePosition(600);
        somn(500);
        servoX.setPosition(0);
        servoCleste2.setPosition(0.5);
        servoY.setPosition(0.32);


    }
    public void firstAutonomous1stIntakeSequence(){
        setSlidePosition(600);
        somn(500);
        servoX.setPosition(0);
        servoCleste2.setPosition(0.5);
        servoY.setPosition(0.3);


    }
    public void firstAutonomous2ndIntakeSequence(){
        int TargetSenzorCleste = 600;
        autoExtend(TargetSenzorCleste);
        while(!isNear()){autoExtend(TargetSenzorCleste++);}
        ElapsedTime time = new ElapsedTime();
        if(time.milliseconds() > 3000){
            setSlidePosition(1600);
            somn(500);
            servoX.setPosition(0);
            servoCleste2.setPosition(0.5);
            servoY.setPosition(0.32);
        }
        servoCleste2.setPosition(0);


    }

    public void firstIntakeSequence(){
        setSlidePosition(600);
        somn(500);
        servoX.setPosition(0);
        servoCleste2.setPosition(0.5);
        servoY.setPosition(0.26);
        int TargetSenzorCleste = 600;
        autoExtend(TargetSenzorCleste * 2);
        while(!isNear()){
            if (gamepad1.share) {
                motorGlisiera2.setPower(0);
                break;
            }
            else autoExtend(TargetSenzorCleste++ * 2);
        }
        servoCleste2.setPosition(0);

    }

    public void secondIntakeSequence() {
        servoY.setPosition(0.6);
        somn(200);
        servoX.setPosition(0.93);
        somn(100);
        servoY.setPosition(0.7);
        somn(500);
        setSlidePosition(760);
        while(isGoing(760));
        somn(100);
        servoX.setPosition(1);
        somn(100);
        servoY.setPosition(0.92);
        somn(500);
        servoCleste2.setPosition(0.3);
        somn(500);
        servoY.setPosition(0.72);
        somn(600);
        servoCleste2.setPosition(0);
        somn(300);
        servoY.setPosition(0.4);
        somn(400);
        servoX.setPosition(0);

    }

    public void manualLevel(double manualTarget) {
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

    public boolean isNear() {
        if (senzorCleste.getDistance(DistanceUnit.CM) <= 5){
            return true;
        }
        return false;
    }

}
