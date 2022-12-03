package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.subsystems.Glisiera;
import org.firstinspires.ftc.teamcode.drive.subsystems.GlisieraDemo;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDriveCh;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Odometrie;

public class Robot {
    private  boolean initialize;
    public MecanumDriveCh drive;
    public GlisieraDemo glisieraDemo;
//    public Odometrie odometrie;

    public Robot (HardwareMap hardwareMap){
        initialize = true;
        drive = new MecanumDriveCh(hardwareMap);
        glisieraDemo = new GlisieraDemo(hardwareMap);
//        odometrie = new Odometrie(hardwareMap);
        initialize = false;
    }
    public boolean isInitialize() {return initialize;}
}