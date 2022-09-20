package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDriveCh;

public class Robot {
    private  boolean initialize;
    public MecanumDriveCh drive;
//    public Odometrie odometrie;

    public Robot (HardwareMap hardwareMap){
        initialize = true;
        drive = new MecanumDriveCh(hardwareMap);
//        odometrie = new Odometrie(hardwareMap);
        initialize = false;
    }
    public boolean isInitialize() {return initialize;}
}
