// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="Autonomous Program", group="Exercises")
@Disabled
public class AutonomousVuforia extends LinearOpMode
{

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            "Ac5+EOX/////AAABmYZoDckxSUV7iTj4MtRwiJpvftO8hpoEtcxmCLD\n"
                    + "olUOn81SCtt0u8igYx5S9Gz9UpcHI66Vto0Wy1IhFoZ4J36MXjIwxdHCb/81+4+4DhbQ2tWq9Xisz4NvAu2l1IN8uj6\n"
                    + "qvmjmg02YfS7+REk+/NxVrS15d2fvFh7lE9RMlLtMwgkK9903e2mgxf48yL9IQMXoTfBJhY3X4cSKSzz4XGKDjqvIXn\n"
                    + "Ad47NYB8TXuOwY0N8bL9+jPNPaw3E2SgOU2imUU6kCAQvrUPF24AI1FqtvlhZbeYLe/EQVJaC2fqcODw2Xp5pxj1h4l\n"
                    + "S6tGXQqRZ1we0i4Wf/S+1/A4GDcb3B7hfpkRJP6AYvLxisBlP2qj";


    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .3, correction;
    static final double     COUNTS_PER_MOTOR_REV    = 2240;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    private static final float mmPerInch        = 25.4f;
    //private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    //private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    //private static final float bridgeZ = 6.42f * mmPerInch;
    //private static final float bridgeY = 23 * mmPerInch;
    //private static final float bridgeX = 5.18f * mmPerInch;
    //private static final float bridgeRotY = 59;                                 // Units are degrees
    //private static final float bridgeRotZ = 180;

    // Initialization parameters for 15343
    //String xyz = "z";
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor strafeMotor = null;
    private DcMotor liftMotor = null;
    private Servo foundationServo1 = null;
    private Servo foundationServo2 = null;

    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private String DObject = null;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        //INITIATE MOTORS AND SERVOS
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        leftMotor = hardwareMap.dcMotor.get("Left_Motor");
        rightMotor = hardwareMap.dcMotor.get("Right_Motor");
        strafeMotor = hardwareMap.dcMotor.get("Strafe_Motor");
        liftMotor = hardwareMap.dcMotor.get("Lift_Motor");
        foundationServo1 = hardwareMap.servo.get("Foundation_Servo1");
        foundationServo2 = hardwareMap.servo.get("Foundation_Servo2");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        strafeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        strafeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //reverse the motors so the robot drives straight


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters1.vuforiaLicenseKey = VUFORIA_KEY;
        parameters1.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters1);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

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

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters1.cameraDirection);
        }

        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
                DObject= trackable.getName();
                break;
            }
        }
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(300);
        if (targetVisible) {
            // Do all autonomous actions here
            if (DObject == "Blue Perimeter 2") {
                //do upper left corner actions (L3-37)
                //Move Up 12 inches using strafe wheel
                //Move forward 24 inches
                //Deploy claws
                //Pull back 24 inches
                //Pull back claws
                //Move down 48 inches to park
                telemetry.addData("Visible Target", DObject);
                telemetry.update();
            } else if (DObject == "Blue Perimeter 1"){
                //do bottom left corner actions (BB8)
                //Move up 8 inches
                //Move forward 36 inches
                //
                telemetry.addData("Visible Target", DObject);
                telemetry.update();
            } else if (DObject == "Red Perimeter 1"){
                //do upper right corner actions (c3po)
                //Move up 12 inches using the strafe wheel
                //Move forward 24 inches
                //Deploy claws
                //Pull back 24 inches
                //Pull back claws
                //Move down 48 inches to park
                telemetry.addData("Visible Target", DObject);
                telemetry.update();
            } else if (DObject == "Red Perimeter 2"){
                //do bottom right corner actions (yellow machine)
                telemetry.addData("Visible Target", DObject);
                telemetry.update();
            } else if (DObject == "Stone Target"){
                telemetry.addData("Visible Target", DObject);
                telemetry.update();
            }else {
                telemetry.addData("Visible Target", DObject);
                telemetry.update();
            }
        }
        else {
            telemetry.addData("Visible Target", "none");
            telemetry.update();
        }

        //START MOVING
        /*leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        foundationMovement();

        foundationServo1.setPosition(1);
        foundationServo2.setPosition(1);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        foundationMoveBack();
         */
    }

    //method used to move the robot to the desired position
 /*   public void park() {
        int leftInches = 36;
        int rightInches = 36;
        int newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        while(opModeIsActive()) {
        correction = checkDirection();

        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.addData("Right Motor", rightMotor.getCurrentPosition());
        telemetry.addData("Left Motor", leftMotor.getCurrentPosition());
        telemetry.addData("Strafe Motor", strafeMotor.getCurrentPosition());
        telemetry.update();

        leftMotor.setPower(power - correction);
        rightMotor.setPower(power + correction);
        }
    }
    public void foundationMovement() {
        //DEFINE MOVEMENT VALUES:
        boolean isActive = true;

        int leftInches = 40;
        int rightInches = 40;
        int newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        //MOVE TO FOUNDATION:
        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(isActive) {
            if(leftMotor.getCurrentPosition() < newLeftTarget && rightMotor.getCurrentPosition() < newRightTarget) {
                // Use gyro to drive in a straight line.
                correction = checkDirection();

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("Right Motor", rightMotor.getCurrentPosition());
                telemetry.addData("Left Motor", leftMotor.getCurrentPosition());
                telemetry.addData("Strafe Motor", strafeMotor.getCurrentPosition());
                telemetry.update();

                leftMotor.setPower(power - correction);
                rightMotor.setPower(power + correction);
            } else {
                isActive = false;
            }
        }


    }
    public void foundationMoveBack() {
        boolean isActive = true;

        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(isActive) {
            if(leftMotor.getCurrentPosition() > 0 && rightMotor.getCurrentPosition() > 0){


                // Use gyro to drive in a straight line.
                correction = checkDirection();

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("Right Motor", rightMotor.getCurrentPosition());
                telemetry.addData("Left Motor", leftMotor.getCurrentPosition());
                telemetry.addData("Strafe Motor", strafeMotor.getCurrentPosition());
                telemetry.update();

                leftMotor.setPower((power - correction));
                rightMotor.setPower((power - correction));
            }
            else {
                isActive = false;
            }
        }
    }
    */

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
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

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}