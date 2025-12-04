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

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="IN PROGRESS Blue Auto (6 artifact)", group="Auto")
public class NewBlueAuto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    // Declare drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Declare end-effector members
    private DcMotor intake = null;
    private DcMotorEx catapult1 = null;
    private DcMotorEx catapult2 = null;
    private DcMotor foot = null;

    // motor power 1 = 100% and 0.5 = 50%
    // negative values = reverse ex: -0.5 = reverse 50%
    private double INTAKE_IN_POWER = 0.75;
    private double INTAKE_OUT_POWER = -0.9;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    private double FOOT_UP_POWER = 1.0;
    private double FOOT_DOWN_POWER = -0.85;
    private double FOOT_OFF_POWER = 0.0;
    private double footPower = FOOT_OFF_POWER;

    private double CATAPULT_UP_POWER         = -1.0;
    private double CATAPULT_DOWN_POWER       = 1.0;
    private double CATAPULT_SET_TARGET_POWER = 0.75;

    private enum CatapultModes {
        UP("UP"), DOWN("DOWN"), BRAKE("BRAKE");

        CatapultModes(String up) {
        }
    }

    private CatapultModes pivotMode;

    private enum FootMode {UP, DOWN, BRAKE}

    private FootMode footmode;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotorEx.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotorEx.class, "catapult2");
        foot = hardwareMap.get(DcMotor.class, "foot");

        // reset both encoders, assuming they exist. This fork uses the encoders on both motors, so make sure they are plugged in!
        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed
        // to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        // set direction of wheel motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // set direction of subsystem motors
        intake.setDirection(DcMotor.Direction.REVERSE);
        catapult1.setDirection(DcMotor.Direction.FORWARD); // Backwards should pivot DOWN, or in the stowed position.
        catapult2.setDirection(DcMotor.Direction.REVERSE);
        foot.setDirection(DcMotor.Direction.REVERSE); // Backwards should should stay UP, or in the stowed position

        // set initial subsystem behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        foot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        while (opModeIsActive()) {

            // Drive slightly away from the wall
            leftFrontDrive.setPower(-0.5);
            rightFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
            rightBackDrive.setPower(-0.5);

            sleep(250);

            // Stop the robot
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Drop the catapult down
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);

            sleep(1000);

            // launch the artifacts
            catapult1.setPower(CATAPULT_UP_POWER);
            catapult2.setPower(CATAPULT_UP_POWER);

            sleep(500);

            // shut the catapult off
            catapult1.setPower(0);
            catapult2.setPower(0);

            // drive back
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(-1);
            leftBackDrive.setPower(-1);
            rightBackDrive.setPower(-1);

            sleep(800);

            // brake briefly
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            sleep(100);

            // rotate 1st
            leftBackDrive.setPower(-1);
            leftFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            rightFrontDrive.setPower(1);

            sleep(225);

            // brake briefly
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            sleep(100);

            // intake on
            intake.setPower(0.7);

            // Drive forward
            leftBackDrive.setPower(0.75);
            leftFrontDrive.setPower(0.75);
            rightBackDrive.setPower(0.75);
            rightFrontDrive.setPower(0.75);

            sleep(1100);

            // stop the robot
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);

            sleep(2000);

            // shut the intake off
            intake.setPower(INTAKE_OFF_POWER);

            // back up to go to the goal
            leftBackDrive.setPower(-0.75);
            leftFrontDrive.setPower(-0.75);
            rightBackDrive.setPower(-0.75);
            rightFrontDrive.setPower(-0.75);

            sleep(900);

            // brake briefly
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            sleep(100);

            // rotate 2nd
            leftBackDrive.setPower(1);
            leftFrontDrive.setPower(1);
            rightBackDrive.setPower(-1);
            rightFrontDrive.setPower(-1);

            sleep(175);

            // brake briefly
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            sleep(100);

            // drive to the goal
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(1);
            leftBackDrive.setPower(1);
            rightBackDrive.setPower(1);

            sleep(950);

            // Stop the robot
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            sleep(500);

            // rock stuck artifacts into the catapult
            intake.setPower(0.9);
            leftBackDrive.setPower(-0.75);
            leftFrontDrive.setPower(-0.75);
            rightBackDrive.setPower(-0.75);
            rightFrontDrive.setPower(-0.75);
            sleep(250);

            // brake briefly
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            sleep(250);

            leftBackDrive.setPower(0.75);
            leftFrontDrive.setPower(0.75);
            rightBackDrive.setPower(0.75);
            rightFrontDrive.setPower(0.75);
            sleep(250);

            // brake briefly
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            sleep(250);

            // shut the intake off
            intake.setPower(INTAKE_OFF_POWER);

            // Drop the catapult down
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);

            sleep(1000);

            // launch the artifacts
            catapult1.setPower(CATAPULT_UP_POWER);
            catapult2.setPower(CATAPULT_UP_POWER);

            sleep(500);

            // shut the catapult off
            catapult1.setPower(0);
            catapult2.setPower(0);

            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();

            requestOpModeStop();
        }

    }
}