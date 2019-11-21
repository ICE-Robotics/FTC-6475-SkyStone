package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public DcMotor leftDrive1 = null;
    public DcMotor leftDrive2 = null;
    public DcMotor rightDrive1 = null;
    public DcMotor rightDrive2 = null;
    public Servo leftServo = null;
    public Servo rightServo = null;
    public Servo armLeft = null;
    public Servo armRight = null;
    public BNO055IMU imu;

    private HardwareMap map = null;

    public RobotHardware(){

    }

    public void init(HardwareMap hMap){
        map = hMap;
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = map.get(BNO055IMU.class, "imu");
        leftDrive1  = map.get(DcMotor.class, "LD1");
        leftDrive2  = map.get(DcMotor.class, "LD2");
        rightDrive1 = map.get(DcMotor.class, "RD1");
        rightDrive2 = map.get(DcMotor.class, "RD2");
        leftServo = map.get(Servo.class, "LeftServo");
        rightServo = map.get(Servo.class, "RightServo");
        armRight = map.get(Servo.class, "ArmRight");
        armLeft = map.get(Servo.class, "ArmLeft");

        imu.initialize(parameters);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.FORWARD);

    }
}
