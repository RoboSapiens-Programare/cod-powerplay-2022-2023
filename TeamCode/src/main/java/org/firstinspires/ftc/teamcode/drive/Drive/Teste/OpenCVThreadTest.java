package org.firstinspires.ftc.teamcode.drive.Drive.Teste;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Ratoi;
import org.firstinspires.ftc.teamcode.drive.localization.OpenCVThread;

@TeleOp(name = "OpenCV Thread Test mor", group = "test")
public class OpenCVThreadTest extends LinearOpMode {
    public OpenCVThread openCV;
    public Ratoi.Location location;
    public ElapsedTime timer;
    public static int MAX_MILISECONDS = 4000;
    public String test = "valoare initiala";

    @Override
    public void runOpMode() throws InterruptedException {
        openCV = new OpenCVThread(hardwareMap);
        location = Ratoi.Location.RIGHT;

        openCV.start();

        timer = new ElapsedTime();
        telemetry.addData("has initialised", "yes");
        telemetry.update();

        waitForStart();

        timer.startTime();

        while (openCV.getLocation().equals(Ratoi.Location.RIGHT) && timer.milliseconds() < MAX_MILISECONDS){
            location = openCV.getLocation();
            telemetry.addData("Stai ma usor", "");
            telemetry.update();
        }

        while (opModeIsActive()){
            telemetry.addData("Location: ", openCV.getLocation());
            telemetry.update();
        }

        try {
            openCV.finalize();
        }catch (Throwable throwable){
            throwable.printStackTrace();
        }
    }
}
