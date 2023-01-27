package org.firstinspires.ftc.teamcode.drive.subsystems;

import static org.firstinspires.ftc.teamcode.drive.opmode.TestEncoder.POWER;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Glisiera {
    public DcMotor motorGlisiera1;
    public Servo cleste;
    public double manualTarget = 0;

    public Glisiera(HardwareMap hardwareMap){
        PhotonCore.enable();
        PhotonCore.CONTROL_HUB.disengage();
        motorGlisiera1 = hardwareMap.dcMotor.get("motorGlisiera1");
        cleste = hardwareMap.servo.get("servoCleste");

        //Motor initialization
        motorGlisiera1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlisiera1.setDirection(DcMotorSimple.Direction.REVERSE);

        cleste.setDirection(Servo.Direction.FORWARD);
    }

    public void strangeCleste(){
        cleste.setPosition(0);
    }

    public void desfaCleste(){
        cleste.setPosition(0.5);
    }

    public void setPower(double power){
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisiera1.setPower(power);
    }

    public void manualLevel(double manualTarget) {
        motorGlisiera1.setTargetPosition((int) manualTarget);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera1.getCurrentPosition() > manualTarget)
        {
            motorGlisiera1.setPower(POWER);
        }
        else {
            motorGlisiera1.setPower(-POWER);
        }
    }

    public void setLevel(int target, double multiplier){
        motorGlisiera1.setTargetPosition(-target);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera1.getCurrentPosition() > -target)
            motorGlisiera1.setPower(-POWER);
        else motorGlisiera1.setPower(POWER * multiplier);
    }
    public void setLevel(int target){
        motorGlisiera1.setTargetPosition(-target);
        motorGlisiera1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorGlisiera1.getCurrentPosition() > -target)
            motorGlisiera1.setPower(-POWER);
        else motorGlisiera1.setPower(POWER);
    }

}


