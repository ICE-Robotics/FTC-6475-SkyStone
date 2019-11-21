package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDriveByEncoder_Linear;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name="Red Skystone", group="Linear Opmode")

public class MoveToSkystoneRed extends LinearOpMode {
    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();   // Use a Pushbot's hardware
    SkystoneDectection vuforia = new SkystoneDectection();
    PushbotAutoDriveByEncoder_Linear drive = new PushbotAutoDriveByEncoder_Linear();

    private ElapsedTime runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.32;
    static final double     TURN_SPEED = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        vuforia.init();
        vuforia.initVuforia();
        vuforia.initTfod();

        /* Detect location of Skystone */
        switch (determinePosition()) {
            case 0:
                telemetry.addData("Far Left", "Complete");
                drive.encoderDrive(FORWARD_SPEED, 1, 1, 5);
                break;
            case 1:
                telemetry.addData("Middle", "Complete");
                drive.encoderDrive(FORWARD_SPEED, 1, 1, 5);
            case 2:
                telemetry.addData("Far Right", "Complete");
                drive.encoderDrive(FORWARD_SPEED, 1, 1, 5);
                break;
            default:
                drive.encoderDrive(0, 0, 0, 0);
                break;
        }

        telemetry.update();
    }

    /* Position 0 -> Far Left
     *  Position 1 -> Middle
     *  Position 2 -> Far Right
     */
    private int determinePosition() {
        int counter = vuforia.getSkystoneCounter();
        float leftPos = vuforia.getSkystoneLeft();
        if (counter == 0) {
            return 1;
        } else if (leftPos < 500) {
            return 0;
        } else if (leftPos > 500) {
            return 2;
        } else {
            return 3; // Non detected
        }

        /*
        right: 920 L
        left: 270 L
         */
    }

}
