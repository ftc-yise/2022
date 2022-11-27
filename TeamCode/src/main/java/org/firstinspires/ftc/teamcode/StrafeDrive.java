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

package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Drive program", group="Linear Opmode")
public class StrafeDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    //private Rev2mDistanceSensor distanceSensorRight = null;
    //private Rev2mDistanceSensor distanceSensorLeft = null;

    private Servo coneGrabber = null;

    public float speedMultiplier = 1;

    public float cone = 1;  // variable used to set the value for the cone heights and incremental down values

    public boolean canSwitchModes = true;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");

        coneGrabber = hardwareMap.get(Servo.class, "cone_grabber");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //distanceSensorRight = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor_right");
        //distanceSensorLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor_left");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            //double distanceRight = distanceSensorRight.getDistance(DistanceUnit.CM);
            //double distanceLeft = distanceSensorLeft.getDistance(DistanceUnit.CM);
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double vertical = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double horizontal = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = vertical + horizontal + turn;
            double rightFrontPower = vertical - horizontal - turn;
            double leftBackPower = vertical - horizontal + turn;
            double rightBackPower = vertical + horizontal - turn;

            /*leftSlide.setPower(-gamepad2.left_stick_y);
            rightSlide.setPower(-gamepad2.left_stick_y);*/


            //Encoder slide
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                leftSlide.setTargetPosition(1950); // high pole position based on string length
                rightSlide.setTargetPosition(1950);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                leftSlide.setTargetPosition(0);
                rightSlide.setTargetPosition(0);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                cone = 0;
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                leftSlide.setTargetPosition(850);  //low pole position
                rightSlide.setTargetPosition(850);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                leftSlide.setTargetPosition(1400);  //mid pole position
                rightSlide.setTargetPosition(1400);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else if (gamepad1.x || gamepad2.x) {
                leftSlide.setTargetPosition(75);  //GROUND JUNCTION hover
                rightSlide.setTargetPosition(75);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } if (cone < 1 ) {  //checks to see if the arm is already at zero and doesn't try to move down anymore
                cone = 0;
            }else if (gamepad2.left_bumper) {
                cone = cone - 8;
                leftSlide.setTargetPosition((int) (cone + 50));  //stack incrementally move arm down by 50 on each press
                rightSlide.setTargetPosition((int) (cone + 50));
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }else if (gamepad2.right_bumper) {
                cone = 300;
                leftSlide.setTargetPosition((int) (300));  //stack height for 5 cones
                rightSlide.setTargetPosition((int) (300));
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1); }







                if (!leftSlide.isBusy() || !rightSlide.isBusy()) {
                    //stop the slide and keep it from holding position
                    leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftSlide.setPower(0.05);
                    rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightSlide.setPower(0.05);
                }

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

                if (gamepad1.a || gamepad2.a) {
                    coneGrabber.setPosition(Servo.MIN_POSITION);
                } else if (gamepad1.b || gamepad2.b) {
                    coneGrabber.setPosition(Servo.MAX_POSITION);
                }

                // Send calculated power to wheels
                if (gamepad1.y && (speedMultiplier == 1) && canSwitchModes) {
                    speedMultiplier = 0.75f;
                } else if (gamepad1.y && (speedMultiplier == 0.75f) && canSwitchModes) {
                    speedMultiplier = 1f;
                }

                if (gamepad1.y) {
                    canSwitchModes = false;
                } else {
                    canSwitchModes = true;
                }

                leftFrontDrive.setPower(leftFrontPower * speedMultiplier);
                rightFrontDrive.setPower(rightFrontPower * speedMultiplier);
                leftBackDrive.setPower(leftBackPower * speedMultiplier);
                rightBackDrive.setPower(rightBackPower * speedMultiplier);

                /*if (gamepad1.left_trigger >= 0.8) {  //CAUSING LAG AND WHEN DISCONNECTS IT STOPS THE CODE BRING THIS OUT TO ANOTHER CLASS AND ADD ERROR CODE HANDELING
                    if (distanceLeft < 20 && distanceRight < 20) {
                        leftFrontDrive.setPower(0);
                        rightBackDrive.setPower(0);
                        leftBackDrive.setPower(0);
                        rightFrontDrive.setPower(0);
                    } else if (distanceRight < 20) {
                        leftFrontDrive.setPower(0.3);
                        rightBackDrive.setPower(0.3);
                        leftBackDrive.setPower(-0.3);
                        rightFrontDrive.setPower(-0.3);
                    } else if (distanceLeft < 20) {
                        leftFrontDrive.setPower(-0.3);
                        rightBackDrive.setPower(-0.3);
                        leftBackDrive.setPower(0.3);
                        rightFrontDrive.setPower(0.3);
                    } else if ((distanceRight < 35 && distanceRight > 20) || (distanceLeft < 35 && distanceLeft > 20)) {
                        leftFrontDrive.setPower(0.4);
                        rightBackDrive.setPower(0.4);
                        leftBackDrive.setPower(0.4);
                        rightFrontDrive.setPower(0.4);
                    }*/
                //}

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("HeightL: ", leftSlide.getCurrentPosition());
                telemetry.addData("HeightR: ", rightSlide.getCurrentPosition());
                telemetry.addData("Left Encoder:", leftFrontDrive.getCurrentPosition());
                telemetry.addData("Rear Encoder:", rightFrontDrive.getCurrentPosition());
                telemetry.addData(" Right Encoder:", -rightBackDrive.getCurrentPosition());
            //telemetry.addData("Distance left: ", distanceLeft);
                //telemetry.addData("Distance right: ", distanceRight);
                telemetry.update();
                telemetry.addData("cone: ", cone);
            }
        }
    }

