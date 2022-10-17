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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Beeline go brrr", group="Linear Opmode")
public class BeelineDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    float speedMultiplier = 1;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double drive;
            double turn;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            if (gamepad1.a && (speedMultiplier == 1)) {
                speedMultiplier = 0.35f;
            } else if (gamepad1.a && (speedMultiplier == 0.35f)){
                speedMultiplier = 1f;
            }

            if (gamepad1.right_trigger > .75)


            if (gamepad2.y){
                leftPower=-leftPower;
            }
            if (gamepad2.x){
                rightPower=-rightPower;
            }
            if (gamepad2.b){
               leftPower = -leftPower;
               rightPower = -rightPower;
            }
            if (gamepad1.right_bumper){
                terminateOpModeNow();
            }

            if (gamepad2.a) {
                terminateOpModeNow();
            }

            /*if (gamepad2.right_trigger > .75 && (speedMultiplier == 1)) {
                speedMultiplier = 0.15f;
            } else if (gamepad1.a && (speedMultiplier == 0.15f)){
                speedMultiplier = 1f;
            }*/
            if (gamepad2.right_bumper){
                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            }

            if (gamepad2.left_bumper){
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            }
            if (gamepad2.start && (speedMultiplier == 1)) {
                speedMultiplier = 0f;
            } else if (gamepad1.back && (speedMultiplier == 0f)){
                speedMultiplier = .5f;
            }




            leftFrontDrive.setPower(leftPower * speedMultiplier);
            rightFrontDrive.setPower(rightPower * speedMultiplier);
            leftBackDrive.setPower(leftPower * speedMultiplier);
            rightBackDrive.setPower(rightPower * speedMultiplier);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
