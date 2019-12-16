package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareRobot {
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor strafeMotor = null;
    public DcMotor liftMotor = null;
    public Servo armServo = null;
    public Servo clawServo = null;
    public Servo foundationServo1, foundationServo2;
    BNO055IMU imu;
    HardwareMap hwMap = null;

}
