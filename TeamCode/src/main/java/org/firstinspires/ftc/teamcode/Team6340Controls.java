package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

import static java.lang.Math.abs;

/**
 * Created by Team ICE 6340 on 10.08.18.
 */

public abstract class Team6340Controls extends LinearOpMode {
    //Initialize and instantiate vuforia variables
    OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "ATrCz8D/////AAABmc3XlIert0HKlboeCjVYhDgsktWr4sPPFzPH3bIJNybjrZFIe+fmydvlsTyXPBy/H1tiSPZYsX86W4u+JDVP29i1kqzhJZBu6SbsYrAVRkbfdKFEiQVzs+yPWllNl3fPXTHVAg5enqTRuAJgCbnomUvm2Sc9zQq827k9dKuxsu77CTxjOFgWeH7dG2KiBu0Td9+NnSkcxB246k8kLz174CxosFeNmH72KchajgkcSg9evq9bWTPQnzmvZ57zUoPrYpnF3coF/8OZilDstMpqiEC3a7WFge00WSuvY9z8IAQY4j/gxMViG0vA5VFxe1J5b+O0VGJiZyV6JJlEhKMXXcmBes06uRfn3m1SDsGKaYC5";


    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    TFObjectDetector tfod;
    protected static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    protected static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    protected static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    protected static final String POSITION_LEFT = "Left";
    protected static final String POSITION_CENTER = "Center";
    protected static final String POSITION_RIGHT = "Right";

    //Initialize elapsed time object
    protected ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    protected BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double absoluteAngle, power = .30, correction;

    // Orientation and acceleration variables from the built in 9-axis accelerometer
    protected Orientation angles;
    protected Acceleration gravity;

    //ROBOT HARDWARE
    //Instantiate chassis motors

    protected DcMotorEx leftMotor;
    protected DcMotorEx rightMotor;
    protected DcMotorEx liftMotor;
   // protected DcMotorEx bucketMotor;
//    protected DcMotorEx armRotationMotor;
////    protected DcMotorEx extendMotor;
////    protected DcMotorEx intakeMotor;


    //Instantiate servos
    protected Servo marker;


    //Instantiate sensors
    ColorSensor blueSensorColor;

    //Initlize encoder variables
    protected double COUNTS_PER_MOTOR_DRIVE = 1120;    // Rev Hex Motor 288
    protected double COUNTS_PER_MOTOR_LIFT = 1120;    // andyMark 40
    protected double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    protected double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    protected double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_DRIVE * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    protected double COUNTS_PER_INCH_LIFT = (COUNTS_PER_MOTOR_LIFT) / (.677 * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = .3;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = .3;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 2.5;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = .010;     // .02 Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = .010;     // .05 Larger is more responsive, but also less stable

    //Initialize Vuforia variables
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    /**
     * Initializes all of the motors and servos.
     * <b>Be sure to call this before waitForStart()</b>
     */
    protected void initializeHardware() {
        //Give the OK message
        telemetry.addData("Status", "Initializing hardware");
        telemetry.update();

        //Initialize robot hardware
        //Begin with the chassis
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");
        liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotor");
        //bucketMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "bucketMotor");
//        armRotationMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "armRotationMotor");
//        extendMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "extendMotor");
//        intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakeMotor");
        //Reset the encoders on the chassis to 0
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        //bucketMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        armRotationMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Set the motor modes
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        //bucketMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        armRotationMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Reverse the right motors so all motors move forward when set to a positive speed.
        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        armRotationMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bucketMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize the servos
       marker = hardwareMap.get(Servo.class, "marker");


        //Initialize sensors
        //blueSensorColor = hardwareMap.get(ColorSensor.class, "BlueColorSensor");



        //Initialize Gryo
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parametersG = new BNO055IMU.Parameters();
        parametersG.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersG.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersG.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersG.loggingEnabled = true;
        parametersG.loggingTag = "IMU";
        parametersG.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersG);




                telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Set up our telemetry dashboard
        composeTelemetry();
        // Set the g

    }

    /**
     * Initialize the Vuforia localization engine.
     */

    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;  //set camera

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        /*
       Get the assets for Vuforia
       relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
       relicTemplate = relicTrackables.get(0);
       relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        */
    }
    protected void initVuforiaWebcam() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        /*
       Get the assets for Vuforia
       relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
       relicTemplate = relicTrackables.get(0);
       relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        */
    }
    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * Begin Vuforia tracking and caliberate gyro.
     * <b>Call this <i>after</i> waitForStart()</b>
     */
    protected void startAdvancedSensing() {
        //Do some calibration and activation
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 40);
        //Activate Vuforia
        relicTrackables.activate();
    }

    protected void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(() -> {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity = imu.getGravity();
        });
        telemetry.addLine().addData("status", () -> imu.getSystemStatus().toShortString()).addData("calib", () -> imu.getCalibrationStatus().toString());
        telemetry.addLine().addData("heading", () -> formatAngle(angles.angleUnit, angles.firstAngle)).addData("roll", () -> formatAngle(angles.angleUnit, angles.secondAngle)).addData("pitch", () -> formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.addLine().addData("grvty", () -> gravity.toString())

                .addData("mag", () -> String.format(Locale.getDefault(), "%.3f", Math.sqrt(gravity.xAccel * gravity.xAccel + gravity.yAccel * gravity.yAccel + gravity.zAccel * gravity.zAccel)));
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));

    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * Sets the power of both right motors
     *
     * @param power the power of each of the motors
     */
    protected void rightDrive(double power) {
        rightMotor.setPower(power);
    }

    /**
     * Sets the power of both left motors
     *
     * @param power the power of each of the motors
     */
    protected void leftDrive(double power) {
        leftMotor.setPower(power);

    }



    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param timeout  the number of seconds to take control of the autonomous program
     *                 before giving up
     */
    protected void gyroDrive(double speed, double distance, double angle, double timeout) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            leftMotor.setTargetPositionTolerance(100);
            rightMotor.setTargetPositionTolerance(100);
            // start motion.
            speed = Range.clip(speed, -1.0, 1.0);
            leftDrive(speed);
            rightDrive(speed);


            double timeoutTime = runtime.seconds() + timeout;
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (leftMotor.isBusy() && rightMotor.isBusy()) && runtime.seconds() <= timeoutTime) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;
                telemetry.addData("left: ", leftSpeed);
                telemetry.addData("right", rightSpeed);
                telemetry.update();
                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(abs(leftSpeed), abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftDrive(leftSpeed);
                rightDrive(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftDrive(0);
            rightDrive(0);
        }
    }



    //Controls lift

    public void lift(double speed,
                     double liftInches,
                     double timeout) {
        int newLiftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
//TODO Determine why it must be taken times 2 to get rught output
            // Determine new target position, and pass to motor controller
            newLiftTarget = liftMotor.getCurrentPosition() + (int)(liftInches*(COUNTS_PER_INCH_LIFT));
            liftMotor.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPositionTolerance(100);


            // reset the timeout time and start motion.
            runtime.reset();
            liftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (liftMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Target", "%7d", newLiftTarget);
                telemetry.addData("Actual", "%7d", liftMotor.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            liftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

//    public void bucket(double speed,
//                     double bucketInches,
//                     double timeout) {
//        int newBucketTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newBucketTarget = (int) (bucketInches * COUNTS_PER_MOTOR_LIFT / 360);
//            bucketMotor.setTargetPosition(newBucketTarget);
//
//            // Turn On RUN_TO_POSITION
//            bucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            bucketMotor.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeout) &&
//                    (bucketMotor.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1", "Running", newBucketTarget);
//                telemetry.addData("Path2", "Running",
//                        bucketMotor.getCurrentPosition());
//                telemetry.update();
//            }
//        }
//            // Stop all motion;
//            bucketMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            bucketMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
        //armRotation
//    public void IntakeMotor(double speed,
//                     double intakeInches,
//                     double timeout) {
//        int newAngleTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newAngleTarget = intakeMotor.getCurrentPosition() + (int)(intakeInches * COUNTS_PER_MOTOR_LIFT);
//            intakeMotor.setTargetPosition(newAngleTarget);
//
//            // Turn On RUN_TO_POSITION
//            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            intakeMotor.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeout) &&
//                    (intakeMotor.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Arm1",  "Running", newAngleTarget);
//                telemetry.addData("Arm2",  "Running",
//                        intakeMotor.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            intakeMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }
//    public void extenedArm(double speed,
//                           double extenedInches,
//                           double timeout) {
//        int newAngleTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newAngleTarget = extendMotor.getCurrentPosition() + (int)(extenedInches * COUNTS_PER_MOTOR_LIFT);
//            extendMotor.setTargetPosition(newAngleTarget);
//
//            // Turn On RUN_TO_POSITION
//            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            extendMotor.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeout) &&
//                    (extendMotor.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Arm1",  "Running", newAngleTarget);
//                telemetry.addData("Arm2",  "Running",
//                        extendMotor.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            extendMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }
//    public void armRotate2() throws InterruptedException {
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
//            armRotationMotor.setPower(-.2);
//        }
//        armRotationMotor.setPower(0);
//
//    }

//    public void armRotate(double speed,
//                          double rotationAngle,
//                          double timeout) {
//        int newAngleTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newAngleTarget = armRotationMotor.getCurrentPosition() + (int)(rotationAngle * COUNTS_PER_MOTOR_LIFT);
//            armRotationMotor.setTargetPosition(newAngleTarget);
//
//            // Turn On RUN_TO_POSITION
//            armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            armRotationMotor.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeout) &&
//                    (armRotationMotor.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Arm1",  "Running", newAngleTarget);
//                telemetry.addData("Arm2",  "Running",
//                        armRotationMotor.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            armRotationMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            armRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }


    /**
         * Method to drive on a fixed compass bearing (angle), based on encoder counts.
         * Move will stop if either of these conditions occur:
         * 1) Move gets to the desired position
         * 2) Driver stops the opmode running.
         *
         * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
         * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
         * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
         *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
         *                 If a relative angle is required, add/subtract from current heading.
         */
    protected void gyroDrive(double speed, double distance, double angle) {
        gyroDrive(speed, distance, angle, 60);
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {
        gyroTurn(speed, angle, 60);
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed   Desired speed of turn.
     * @param angle   Absolute Angle (in Degrees) relative to last gyro reset.
     *                0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                If a relative angle is required, add/subtract from current heading.
     * @param timeout the number of seconds to take control of the autonomous program
     *                before giving up
     */
    public void gyroTurn(double speed, double angle, double timeout) {
        //Ensure the motors are in the right configuration
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double timeoutTime = runtime.seconds() + timeout;
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && runtime.seconds() < timeoutTime) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftDrive(0);
        rightDrive(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = Range.clip(speed * steer,-1,1);
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftDrive(leftSpeed);
        rightDrive(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(PCoeff * error, -1, 1);


    }






    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;


        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        lastAngles = angles;

        absoluteAngle += deltaAngle;

        return absoluteAngle;

    }

    /**
     * Method to find the Gold Mineral position
     * For details, see following method
     * @see Team6340Controls#getMineralPosition(double)
     *
     */
    protected String getMineralPosition() {
        return getMineralPosition(60);
    }

    /**
     * Method to find the Gold Mineral position with timeout option
     * @param timeout - number of seconds to find the mineral position.
     *                If TensorFlow is unable to find the mineral position by this number of seconds,
     *                then this program will return null.
     * @return String - returns the position of the gold mineral by the given timeout period or null if time runs out
     *                - Will return Left, Right, Center or null
     */
    protected String getMineralPosition(double timeout) {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            //reset the timeout time and start processing
           runtime.reset();

            //run until the given timeout period
            while (opModeIsActive() && (runtime.seconds() < timeout)) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    return POSITION_LEFT;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    return POSITION_RIGHT;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    return POSITION_CENTER;
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
        return null;
    }


    /**
     * Method to find the Gold Mineral position with timeout option
     * @param timeout - number of seconds to find the mineral position.
     *                If TensorFlow is unable to find the mineral position by this number of seconds,
     *                then this program will return null.
     * @return String - returns the position of the gold mineral by the given timeout period or null if time runs out
     *                - Will return Left, Right, Center or null
     */
    protected String getPositionFromLeftTwoMinerals(double timeout) {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            //reset the timeout time and start processing
            runtime.reset();

            //run until the given timeout period
            while (opModeIsActive() && (runtime.seconds() < timeout)) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;

                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                }
                            }

                            // if both are silver and gold is not found, then its x position would be -1.
                            if(goldMineralX == -1) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                return POSITION_RIGHT;
                            }

                            // if there is one gold and one silver, use the x positions to figure out
                            // which mineral is on the left.
                            // if gold has lower x value, then it's on the left.
                            if(goldMineralX < silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                return POSITION_LEFT;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                return POSITION_CENTER;
                            }

                        }
                        telemetry.update();
                    }
                }
            }
        }
        return null;
    }



}
