//package org.firstinspires.ftc.teamcode.drive.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//
//public class Odometrie {
//    public DcMotorEx Odometrie;
//    public Odometrie(HardwareMap hardwareMap){
//        Odometrie = (DcMotorEx) hardwareMap.dcMotor.get("Odometrie");
//        Odometrie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Odometrie.setDirection(DcMotorSimple.Direction.FORWARD);
//        Odometrie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    public void setPower(double power){
//        Odometrie.setPower(0);
//    }
//}
//
