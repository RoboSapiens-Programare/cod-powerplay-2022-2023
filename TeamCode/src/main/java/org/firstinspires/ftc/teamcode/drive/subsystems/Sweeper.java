package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sweeper {
    public DcMotor motorBrat;
    public DcMotor motorSweeper;
    public DcMotor motorCuva;

    public Sweeper(HardwareMap hardwareMap){
        motorBrat = hardwareMap.dcMotor.get("motorBrat");
        motorSweeper = hardwareMap.dcMotor.get("motorSweeper");
        motorCuva = hardwareMap.dcMotor.get("motorCuva");

        //Motor initialization
        motorBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBrat.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorCuva.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCuva.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCuva.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSweeper.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake(){
        motorSweeper.setPower(1);
    }
    public void outtake(){
        motorSweeper.setPower(-1);
    }
    public void sweepIn() {
        motorSweeper.setPower(0.7);
    }
    public void sweepOut() {
        motorSweeper.setPower(-0.5);
    }
    public void stopSweep(){
        motorSweeper.setPower(0);
    }

    public void bratUp(){
        motorBrat.setTargetPosition(950);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setPower(1);
    }

    public void bratDown(){
        motorBrat.setTargetPosition(10);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setPower(-0.2);
    }

    public void levelOne(){
        motorCuva.setTargetPosition(950);
        motorCuva.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorCuva.setPower(-0.5);

        motorBrat.setTargetPosition(0);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setPower(-0.2);
    }

    public void levelTwo(){
        motorCuva.setTargetPosition(700);
        motorCuva.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorCuva.getCurrentPosition() > 700)
            motorCuva.setPower(0.5);
        else motorCuva.setPower(-0.5);

        motorBrat.setTargetPosition(0);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setPower(-0.2);
    }

    public void levelThree(){
        motorCuva.setTargetPosition(600);
        motorCuva.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorCuva.setPower(-0.4);

        motorBrat.setTargetPosition(1200);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setPower(0.5);
    }

    public void culcatCuva() {
        motorCuva.setTargetPosition(1080);
        motorCuva.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorCuva.setPower(-0.5);

        motorBrat.setTargetPosition(0);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setPower(-0.2);
    }

    public void resetCuva() {
        motorCuva.setTargetPosition(0);
        motorCuva.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorCuva.setPower(0.5);

        motorBrat.setTargetPosition(0);
        motorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBrat.setPower(-0.2);
    }
}