/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


@Autonomous(name = "The Real Autonomous", group = "Linear Opmode")
//@Disabled
public class AutonomousVuforia extends LinearOpMode {

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Ac5+EOX/////AAABmYZoDckxSUV7iTj4MtRwiJpvftO8hpoEtcxmCLD\n"
                    + "olUOn81SCtt0u8igYx5S9Gz9UpcHI66Vto0Wy1IhFoZ4J36MXjIwxdHCb/81+4+4DhbQ2tWq9Xisz4NvAu2l1IN8uj6\n"
                    + "qvmjmg02YfS7+REk+/NxVrS15d2fvFh7lE9RMlLtMwgkK9903e2mgxf48yL9IQMXoTfBJhY3X4cSKSzz4XGKDjqvIXn\n"
                    + "Ad47NYB8TXuOwY0N8bL9+jPNPaw3E2SgOU2imUU6kCAQvrUPF24AI1FqtvlhZbeYLe/EQVJaC2fqcODw2Xp5pxj1h4l\n"
                    + "S6tGXQqRZ1we0i4Wf/S+1/A4GDcb3B7hfpkRJP6AYvLxisBlP2qj";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 13.4f;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double power = .3;
    private double correction = 0;


    // Initialization parameters for 15343
    //CONTAINS ALL METHODS AND VARIABlES TO BE EXTENDED BY OTHER AUTON CLASSES
    private static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = .6;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double COUNTS_PER_INCH_STRAFE = (COUNTS_PER_MOTOR_REV * .66) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor strafeMotor;
    private Servo foundationServo1;
    private Servo foundationServo2;

    private BNO055IMU imu;
    private String positionOfRobot = "";

    private boolean targetVisible = false;
    private String DObject = null;
    private int locationOfSkystoneFromTop;
    private PIDController pidRotate = new PIDController(.003, .00003, 0);
    private PIDController pidDrive = new PIDController(.05, 0.0005, 0.2);


    @Override
    public void runOpMode() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        leftMotor = hardwareMap.dcMotor.get("Left_Motor");
        rightMotor = hardwareMap.dcMotor.get("Right_Motor");
        strafeMotor = hardwareMap.dcMotor.get("Strafe_Motor");
        foundationServo1 = hardwareMap.servo.get("Foundation_Servo1");
        foundationServo2 = hardwareMap.servo.get("Foundation_Servo2");
        ColorSensor sensorColor = hardwareMap.get(ColorSensor.class, "color");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //reset encoder values
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //reverse the motors so the robot drives straight
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();

        parametersIMU.mode = BNO055IMU.SensorMode.IMU;
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        // Class Members
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<>(targetsSkyStone);


        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.


        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        float phoneXRotate = 0;
        float phoneYRotate = 0;
        float phoneZRotate = 0;
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.
        telemetry.addData("Moving", "moving");
        telemetry.update();
        retractClaws();
        moveBack(14);
        if (opModeIsActive()) {
            targetsSkyStone.activate();
            while (!targetVisible) {
                // check all the trackable targets to see which one (if any) is visible.
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;
                        DObject = trackable.getName();
                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.

                    }
                }
                if (targetVisible && opModeIsActive()) {
                    if (Objects.equals(DObject, "Blue Perimeter 2")) {
                        //do upper left corner actions (L3-37)
                        telemetry.addData("Visible Target", DObject);
                        telemetry.update();
                        sleep(1000);
                        targetsSkyStone.deactivate();
                        positionOfRobot = "UpperLeft";
                    } else if (Objects.equals(DObject, "Blue Perimeter 1")) {
                        //do bottom left corner actions (BB8)
                        telemetry.addData("Visible Target", DObject);
                        telemetry.update();
                        sleep(1000);
                        targetsSkyStone.deactivate();
                        positionOfRobot = "BottomLeft";
                    } else if (Objects.equals(DObject, "Red Perimeter 1")) {
                        //do upper right corner actions (c3po)
                        telemetry.addData("Visible Target", DObject);
                        telemetry.update();
                        sleep(1000);
                        targetsSkyStone.deactivate();
                        positionOfRobot = "UpperRight";

                    } else if (Objects.equals(DObject, "Red Perimeter 2")) {
                        //do bottom right corner actions (yellow machine)
                        telemetry.addData("Visible Target", DObject);
                        telemetry.update();
                        sleep(1000);
                        targetsSkyStone.deactivate();
                        positionOfRobot = "BottomRight";

                    } else {
                        telemetry.addData("Visible Target", DObject);
                        telemetry.update();
                        positionOfRobot = "Unknown";
                    }
                } else {
                    telemetry.addData("Visible Target", "none");
                    telemetry.update();
                    positionOfRobot = "Unknown";
                }
                telemetry.update();
            }
            telemetry.update();

            // Disable Tracking when we are done;
            targetsSkyStone.deactivate();
        }

        label:
        while ((!(positionOfRobot.equals("Unknown"))) && opModeIsActive()) {
            switch (positionOfRobot) {
                case "UpperLeft":
                    moveBack(15);
                    moveRightStrafeMotor(15, 0.7);
                    sleep(500);
                    deployClaws();
                    sleep(500);
                    moveForward(34);
                    retractClaws();
                    sleep(500);
                    //Park closest to the perimeter wall
                    moveLeftStrafeMotor(51, 0.75);
                    //or Park closest to the bridge
                    //moveLeftStrafeMotor(24,0.75);
                    //moveBack(28);
                    //moveLeftStrafeMotor(24,0.75);
                    break label;
                case "BottomLeft":
                    moveRightStrafeMotor(13, 0.75);
                    moveBack(12);
                    while (opModeIsActive()) {
                        if (((sensorColor.red() * sensorColor.green()) / (sensorColor.blue() * sensorColor.blue()) < 3)) { //True for skystone
                            locationOfSkystoneFromTop++;
                            telemetry.addData("Skystone", "Yes");
                            telemetry.update();
                            moveForward(8);
                            rotate(175, 0.4);
                            //moveLeftStrafeMotor(5,0.7);
                            telemetry.update();
                            moveForward(20);
                            rotate(85, 0.4);
                            moveLeftStrafeMotor(28, 0.75); //17.5
                            moveForward(24 + (8 * locationOfSkystoneFromTop));
                            moveBack(14);
                            break;
                        } else {
                            moveLeftStrafeMotor(9.4, 0.75);
                            locationOfSkystoneFromTop++;
                            telemetry.addData("Skystone", "Nope");
                            telemetry.update();
                        }
                    }
                    break label;
                case "UpperRight":
                    moveBack(15);
                    moveLeftStrafeMotor(15, 0.7);
                    sleep(500);
                    deployClaws();
                    sleep(500);
                    moveForward(34);
                    retractClaws();
                    sleep(500);
                    moveRightStrafeMotor(51, 0.75);
                    //or Park closest to the bridge
                    //moveRightStrafeMotor(24,0.75);
                    //moveBack(28);
                    //moveRightStrafeMotor(24,0.75);
                    break label;
                case "BottomRight":
                    moveLeftStrafeMotor(12, 0.75);
                    moveBack(12);
                    while (opModeIsActive()) {
                        if (((sensorColor.red() * sensorColor.green()) / (sensorColor.blue() * sensorColor.blue()) < 3)) { //True for skystone
                            locationOfSkystoneFromTop++;
                            telemetry.addData("Skystone", "Yes");
                            telemetry.update();
                            moveForward(8);
                            rotate(-175, 0.4);
                            //moveLeftStrafeMotor(5,0.7);
                            telemetry.update();
                            moveForward(20);
                            rotate(-85, 0.4);
                            moveRightStrafeMotor(28, 0.75); //17.5
                            moveForward(24 + (8 * locationOfSkystoneFromTop));
                            moveBack(14);
                            break;
                        } else {
                            moveRightStrafeMotor(7, 0.75);
                            locationOfSkystoneFromTop++;
                            telemetry.addData("Skystone", "Nope");
                            telemetry.update();
                        }
                    }
                    break label;
            }

        }

    }

    private void moveRightStrafeMotor(double targetInchesStrafe, double power) {
        int newStrafeTarget = (int) (targetInchesStrafe * COUNTS_PER_INCH) - strafeMotor.getCurrentPosition();
        strafeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        strafeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeMotor.setPower(power);
        while (opModeIsActive()) {
            if (strafeMotor.getCurrentPosition() + 50 > newStrafeTarget) {
                strafeMotor.setPower(0);
                break;
            }
        }
        idle();
    }

    private void moveLeftStrafeMotor(double targetInchesStrafe, double power) {
        int newStrafeTarget = (int) ((targetInchesStrafe * COUNTS_PER_INCH_STRAFE) * -1);
        strafeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        strafeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeMotor.setPower(-power);
        while (opModeIsActive()) {
            if (strafeMotor.getCurrentPosition() + 50 < newStrafeTarget) {
                strafeMotor.setPower(0);
                break;
            }
        }
        idle();
    }


    private void deployClaws() {
        foundationServo1.setPosition(1);
        foundationServo2.setPosition(0);
    }

    private void retractClaws() {
        foundationServo1.setPosition(0);
        foundationServo2.setPosition(1);
    }

    private void moveForward(double targetInches) {
        //Update 12/29/19: Added PID Drive
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        //DEFINE MOVEMENT VALUES:
        //MOVE TO FOUNDATION:
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(300);
        int newLeftTarget = (int) (targetInches * COUNTS_PER_INCH);
        int newRightTarget = (int) (targetInches * COUNTS_PER_INCH);


        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (leftMotor.getCurrentPosition() < newLeftTarget && rightMotor.getCurrentPosition() < newRightTarget && opModeIsActive()) {
            // Use gyro to drive in a straight line.

            leftMotor.setPower((power - correction) * 1.5);
            rightMotor.setPower((power + correction) * 1.5);
            correction = pidDrive.performPID(getAngle());
            telemetry.addData("Left Motor", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor", rightMotor.getCurrentPosition());
            telemetry.update();
        }
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        correction = 0;
        pidDrive.reset();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(200);
        idle();

    }

    private void moveBack(double targetInches) {
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(200);
        int newLeftTarget = (int) (targetInches * COUNTS_PER_INCH);
        int newRightTarget = (int) ((targetInches * COUNTS_PER_INCH) * -1);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (leftMotor.getCurrentPosition() < newLeftTarget && rightMotor.getCurrentPosition() > newRightTarget && opModeIsActive()) {
            telemetry.update();
            leftMotor.setPower((power + correction) * -1.5);
            rightMotor.setPower((power - correction) * -1.5);
            correction = pidDrive.performPID(getAngle());
            telemetry.addData("Correction", correction);
            telemetry.addData("Movement", "Forward");
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        correction = 0;
        pidDrive.reset();
        sleep(200);
        idle();
    }


    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //If degrees positive, it will turn left
    //If degrees negative, it will turn right
    //Cannot turn more than 180 degrees
    private void rotate(int degrees, double power) {
// restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        telemetry.addData("Moving", degrees);
        telemetry.update();

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                leftMotor.setPower(power);
                rightMotor.setPower(-power);
                sleep(100);
                telemetry.update();
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                leftMotor.setPower(-power);
                rightMotor.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                leftMotor.setPower(-power);
                rightMotor.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();

    }


}
