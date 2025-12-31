/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Bot Centric TeleOp", group = "Teleop")

public class TeleOpControlLinearOpMode extends LinearOpMode {
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
    private double INTAKE_IN_POWER = 1.0;
    private double INTAKE_OUT_POWER = -0.9;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    private double FOOT_UP_POWER = 0.85;
    private double FOOT_DOWN_POWER = -0.85;
    private double FOOT_OFF_POWER = 0.0;
    private double footPower = FOOT_OFF_POWER;

    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;
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

        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        // same as previous year's loop() code
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //axial = speed, lateral = turn, yaw = strafe
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // ONLY call this once per loop or you will see significant speed issues.
            double catapult1MotorCurrent = catapult1.getCurrent(CurrentUnit.AMPS);
            double catapult2MotorCurrent = catapult2.getCurrent(CurrentUnit.AMPS);

            boolean intakeInButton = gamepad1.left_trigger > 0.2;
            boolean intakeOutButton = gamepad1.left_bumper;

            // This conditional reduces ambiguity when multiple buttons are pressed.
            if (intakeInButton && intakeOutButton) {
                intakeInButton = false;
            }

            boolean footOutButton = gamepad1.a;
            boolean footUpButton = gamepad1.b;
            if (footOutButton && footUpButton) {
                footOutButton = false;
            }

            boolean catapultDownButton = gamepad1.right_trigger > 0.2;

            // DRIVE CODE
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // INTAKE CODE
            if (intakeInButton) {
                intakePower = INTAKE_IN_POWER;
            } else if (intakeOutButton) {
                intakePower = INTAKE_OUT_POWER;
            } else {
                intakePower = INTAKE_OFF_POWER;
            }

            // FOOT CODE
            if (footOutButton) {
                footmode = FootMode.DOWN;
                footPower = FOOT_DOWN_POWER;
            } else if (footUpButton) {
                footmode = FootMode.UP;
                footPower = FOOT_UP_POWER;
            } else {
                footmode = FootMode.BRAKE;
                foot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Determine pivot mode
            if (catapultDownButton) {
                pivotMode = CatapultModes.DOWN;

                catapult1.setPower(CATAPULT_DOWN_POWER);
                catapult2.setPower(CATAPULT_DOWN_POWER);
            } else {
                if (pivotMode == CatapultModes.DOWN) {
                    pivotMode = CatapultModes.UP;

                    catapult1.setPower(CATAPULT_UP_POWER);
                    catapult2.setPower(CATAPULT_UP_POWER);
                } else if (pivotMode == CatapultModes.UP) {
                    if (Math.abs(catapult1.getCurrentPosition()) < 10 || Math.abs(catapult2.getCurrentPosition()) < 10) {
                        // pivotMode = CatapultModes.BRAKE;

                        catapult1.setPower(0);
                        catapult2.setPower(0);
                    }
                }
            }

            // WRITE EFFECTORS - Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            intake.setPower(intakePower);
            foot.setPower(footPower);

            // UPDATE TELEMETRY
            // Show the elapsed game time, wheel power, and other systems power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Intake", "%%4.2f", intake.getPower());
            telemetry.addData("Foot Power", "%4.2f", foot.getPower());
            telemetry.addData("Foot MODE", "%s", footmode);
            telemetry.addData("Catapult1 Current Draw: ", (catapult1MotorCurrent));
            telemetry.addData("Catapult2 Current Draw: ", (catapult2MotorCurrent));
            telemetry.addData("Catapult 1 Encoder: ", catapult1.getCurrentPosition());
            telemetry.addData("Catapult 2 Encoder: ", catapult2.getCurrentPosition());
            telemetry.addData("Catapult MODE", "%s", pivotMode);
            telemetry.update();
        }
    }
}