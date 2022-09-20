package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;

@Autonomous(name="90° turn test", group = "autonomous")
public class AutonomousTurn extends LinearOpMode {

    private Robot robot = null;

    public void initAutonomous(){
        telemetry.addData(">", "Initializing...");
        telemetry.update();
        robot = new Robot(hardwareMap);
        telemetry.addData("has initialised", "yes");
        telemetry.update();

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initAutonomous();

        waitForStart();

        robot.drive.turn(Math.toRadians(90));
    }
}
