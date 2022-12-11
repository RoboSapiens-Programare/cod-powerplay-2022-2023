package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;

@Autonomous(name="Caramida robosap", group = "autonomous")
public class BRICK extends LinearOpMode {
    private Robot robot = null;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
    }
}
