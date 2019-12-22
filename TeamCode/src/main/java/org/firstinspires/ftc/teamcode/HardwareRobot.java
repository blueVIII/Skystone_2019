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
    public void init( HardwareMap ahwMap) {
        hwMap = ahwMap;
        leftMotor = hwMap.dcMotor.get("Left");
        leftMotor = hwMap.dcMotor.get("Left_Motor");
        rightMotor = hwMap.dcMotor.get("Right_Motor");
        strafeMotor = hwMap.dcMotor.get("Strafe_Motor");
        liftMotor = hwMap.dcMotor.get("Lift_Motor");
        armServo = hwMap.servo.get("Arm_Servo");
        clawServo = hwMap.servo.get("Claw_Servo");
        foundationServo1 = hwMap.servo.get("Foundation_Servo1");
        foundationServo2 = hwMap.servo.get("Foundation_Servo2");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);


    }

}
