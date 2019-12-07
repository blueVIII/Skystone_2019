/* Copyright (c) 2017 FIRST. All rights reserved.
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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Slide Drive", group="Linear Opmode")
//@Disabled
public class Forward extends OpMode {
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;

    //DEFINE MOTORS AND SERVOS
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor strafeMotor = null;
    private DcMotor liftMotor = null;
    private Servo armServo = null;
    private Servo clawServo = null;
    private Servo foundationServo1, foundationServo2;

    //DEFINE POWER FOR THE MOTORS
    private int tickCount;
    private double leftPower;
    private double rightPower;
    private double strafePower;
    private double liftPower;
    boolean clawIsOpen = true;
    boolean foundationServoOpen = true;


    // Declare OpMode members.
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftMotor = hardwareMap.dcMotor.get("Left_Motor");
        rightMotor = hardwareMap.dcMotor.get("Right_Motor");
        strafeMotor = hardwareMap.dcMotor.get("Strafe_Motor");
        liftMotor = hardwareMap.dcMotor.get("Lift_Motor");
        armServo = hardwareMap.servo.get("Arm_Servo");
        clawServo = hardwareMap.servo.get("Claw_Servo");
        foundationServo1 = hardwareMap.servo.get("Foundation_Servo1");
        foundationServo2 = hardwareMap.servo.get("Foundation_Servo2");

        clawServo.setPosition(1); //Robot moves to reset position on Initialization.
        //resetting / reversing lift motor
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        // Setup a variable for each drive wheel to save power level for telemetry
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
/*    @Override
    public void start()
    {
        //runtime.reset();
    }*/

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.

        //controls for motors and servos (includes button to slow down the speed)
        if(gamepad1.left_bumper) {
            leftPower  = (gamepad1.left_stick_y) / 1.75;
            rightPower = (gamepad1.right_stick_y) / 1.75;
        } else {
            leftPower  = (gamepad1.left_stick_y);
            rightPower = (gamepad1.right_stick_y);
        }
        strafePower = ((gamepad1.left_stick_x) + (gamepad1.right_stick_x)) / 2;
        liftPower = (gamepad2.left_stick_y) / -1.5;

        // Send calculated power to wheels
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        strafeMotor.setPower(strafePower);
        liftMotor.setPower(liftPower);

        //if(gamepad2.left_bumper) {
        //    clawServo.setPosition(0);
        //} else if (gamepad2.right_bumper) {
        //    clawServo.setPosition(1);
        //}
        //Improved Claw Code
        if(gamepad2.x || gamepad1.dpad_left) {
            clawServo.setPosition(1);
        } else if(gamepad2.b || gamepad1.dpad_right) {
            clawServo.setPosition(0);
        }

        if(gamepad1.x) {
            foundationServo1.setPosition(1);
            foundationServo2.setPosition(0);
        } else if (gamepad1.b) {
            foundationServo1.setPosition(0);
            foundationServo2.setPosition(1);
        }


        //gets current position of servo, if button pressed add .003 to the current position
        double i = armServo.getPosition();
        if(gamepad2.y || gamepad1.dpad_up) {
            i = i+.003;
            armServo.setPosition(i);
        } else if (gamepad2.a || gamepad1.dpad_down) {
            i = i-.003;
            armServo.setPosition(i);
        }

        tickCount = liftMotor.getCurrentPosition();
        //power for lift
        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f), strafe(%.2f), lift (%.2f)", leftPower, rightPower, strafePower, liftPower);
        telemetry.addData("Tick Count", tickCount);
        telemetry.addData("Servo Position", i);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        strafeMotor.setPower(0.0);
        liftMotor.setPower(0.0);
    }
}
