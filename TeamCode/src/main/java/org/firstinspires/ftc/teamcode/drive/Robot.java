package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.teamcode.drive.subsystems.Glisiera;
import org.firstinspires.ftc.teamcode.drive.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDriveCh;
//import org.firstinspires.ftc.teamcode.drive.subsystems.Odometrie;

public class Robot {
    private  boolean initialize;
    public MecanumDriveCh drive;
    public Outtake outtake;
    public Intake intake;
//    public Odometrie odometrie;

    public Robot (HardwareMap hardwareMap){
        initialize = true;
        drive = new MecanumDriveCh(hardwareMap);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
//        odometrie = new Odometrie(hardwareMap);
        initialize = false;
    }
    public boolean isInitialize() {return initialize;}
}