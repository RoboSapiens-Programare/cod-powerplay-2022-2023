package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Ratoi;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.localization.OpenCVThread;

@Autonomous(name="ParkWarehouse Blue", group = "autonomous")
public class ParkWarehouseBlue extends LinearOpMode {

    private Robot robot = null;
    private ElapsedTime timer;

    public OpenCVThread openCV;
    public ElapsedTime opencvTimer;
    public static int MAX_MILISECONDS = 5000;
    private Ratoi.Location finalLocation;

    public void initAutonomous(){
        telemetry.addData(">", "Initializing...");
        telemetry.update();
        robot = new Robot(hardwareMap);

        openCV = new OpenCVThread(hardwareMap);
        finalLocation = Ratoi.Location.RIGHT;

        openCV.start();

        timer = new ElapsedTime();
        telemetry.addData("has initialised", "yes");
        telemetry.update();

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();
    }

    private void mergi(int maxTime, Pose2d pose2d){
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        elapsedTime.startTime();

        while (elapsedTime.milliseconds() < maxTime){
            robot.drive.setDrivePower(pose2d);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initAutonomous();
        waitForStart();

        opencvTimer = new ElapsedTime();

        opencvTimer.startTime();

        timer.startTime();

        while (opencvTimer.milliseconds() < MAX_MILISECONDS){
            telemetry.addData("Location: ", openCV.getLocation());
            telemetry.update();
            finalLocation = openCV.getLocation();
        }

        try {
            openCV.finalize();
        }catch (Throwable throwable){
            throwable.printStackTrace();
        }
        timer = new ElapsedTime();
        timer.startTime();

        mergi(400 , new Pose2d(-0.7, 0, 0));
        robot.drive.turn(Math.toRadians(120));
        mergi(800 , new Pose2d(-1, 0, 0));
    }
}