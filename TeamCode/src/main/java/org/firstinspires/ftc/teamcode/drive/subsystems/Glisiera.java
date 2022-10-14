package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Glisiera {
    private DcMotor motorGlisiera;
    public Glisiera(HardwareMap hardwareMap){
        motorGlisiera = hardwareMap.dcMotor.get("motorGlisiera");

        //Motor initialization
        motorGlisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorGlisiera.setDirection(DcMotorSimple.Direction.FORWARD);
        motorGlisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power){
        motorGlisiera.setPower(power);
    }
}
