package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDriveCh;
import org.firstinspires.ftc.teamcode.drive.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.drive.subsystems.Sweeper;

public class Robot {
    private  boolean initialize;
    public Sweeper sweeper;
    public Carousel carousel;
    public MecanumDriveCh drive;
//    public Odometrie odometrie;

    public Robot (HardwareMap hardwareMap){
        initialize = true;
        sweeper = new Sweeper(hardwareMap);
        carousel = new Carousel(hardwareMap);
        drive = new MecanumDriveCh(hardwareMap);
//        odometrie = new Odometrie(hardwareMap);
        initialize = false;
    }
    public boolean isInitialize() {return initialize;}
}

