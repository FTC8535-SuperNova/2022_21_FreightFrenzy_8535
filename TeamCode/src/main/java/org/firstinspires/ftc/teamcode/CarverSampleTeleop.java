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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


/** This is sample code for an encoder driven arm motor, for the herndon HS scrimmage
 *
 * @version 1.2.5
 * @author Thomas Carver
 */
@TeleOp(name="Sample Teleop for scrim", group="Linear Opmode")
public class CarverSampleTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo armServo = null;
    private DcMotor armMotor = null;
    private CRServo duckServo = null;

    // Final variables cannot be changed once assigned, they are in all caps with underscores unlike other variables
    final static double COUNTS_PER_MOTOR_REV = 384.5; // Encoders measure in clicks, this states how many clicks per full rotation of the motor and can be found in the documentation fot the specific motor
    final static double LENGTH_OF_ARM_CM = 30; // This is A PLACEHOLDER for the length of the arm in cm, needs to be measured.
    final static double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV)/(LENGTH_OF_ARM_CM*Math.PI);







    /** Main OpMode loop, runs until told otherwise,
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {






        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armServo = hardwareMap.get(Servo.class, "claw_servo");
        armMotor = hardwareMap.get(DcMotor.class,"arm_motor");
        duckServo =  hardwareMap.get(CRServo.class, "duck_servo");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // The motor encoders need to be zeroed every time you initialize
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for it to finish resetting
        Thread.sleep(500);

        // You cannot be in DcMotor.RunMode.RUN_TO_POSITION without having a target pos
        armMotor.setTargetPosition(0);

        // All setModes must be done after hardware init
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Some variables for use in the loop, these have set values at the start and as such they are outside of the loop
        double powerMultiplier = 0.75;
        int lastPosition= 0; // The brake position that the robot has set
        boolean isFirstZeroOfStick = true; // Is this the first iteration of the loop where the left stick is zero?
        int targetAddition = 0; // Variable for math
        boolean servoIsRunning = false; // Variable for toggleable servo
        long loopIterations = 0; // How many iterations has the loop done, used to decrease polling rate. Eg. poll every 5 iterations

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // This is an E-Stop or Emergency stop, there is one on the phone, but this one has additional clauses that make it specific to the program, additionally it does not require the operator to put down the controller first
            if (gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1|| gamepad1.left_trigger == 1 && gamepad2.right_trigger == 1){
                RobotLog.addGlobalWarningMessage("E-Stop triggered!"); // This is how you declare warning messages that stick around after program stop

                leftDrive.setPower(0);
                rightDrive.setPower(0);
                armMotor.setPower(0);

                stop(); // The stop method is a part of linearOpMode, does some other things, and can delay stops for up to 100 days if necessary

                ExecutorService executor = Executors.newSingleThreadExecutor();
                executor.shutdownNow(); // Okay, maybe this is a bit overkill, but if the method gets this far, this will force the executor service to terminate the thread

            } // end e-stop if



            // This is to control the servo for the claw
            if (gamepad2.a) {
                armServo.setPosition(0.90);

            } else if (gamepad2.b){
                armServo.setPosition(0.30);

            } // end claw servo if/else if



            // This is to control the CR (continuous rotation) servo for the ducks
            if (gamepad2.x && !servoIsRunning){ // if it's not running this will set its status to running, and move it in the preset direction
                duckServo.setPower(1); // CR servos use setPower rather than setPosition
                duckServo.setDirection(DcMotorSimple.Direction.FORWARD);
                servoIsRunning = true;

            } else if (gamepad2.y && !servoIsRunning){
                duckServo.setPower(1);
                duckServo.setDirection(DcMotorSimple.Direction.REVERSE);
                servoIsRunning = true;

            } else if (servoIsRunning && gamepad2.x || gamepad2.y){ // if it is running, this will set it's status to not running, and stop it
                servoIsRunning=false;
                duckServo.setPower(0);

            } // end duck servo control else if



            if (gamepad1.a){ // This is what I call a "virtual gearshift" it just allows you to change the speed of the robot on the fly for precision movements
                powerMultiplier = 1; // 1 is full speed, you get the idea

            } else if (gamepad1.b){
                powerMultiplier = 0.75;

            } else if (gamepad1.y){
                powerMultiplier = 0.5;

            } else if (gamepad1.x){
                powerMultiplier = 0.2;

            } // end v-gearshift  if/else if/else if/else if



            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  -gamepad1.left_stick_x;

            leftPower    = Range.clip((drive + turn)*powerMultiplier, -1.0, 1.0);
            rightPower   = Range.clip((drive - turn)*powerMultiplier, -1.0, 1.0);



            // Oh boy here comes the encoder stuff...


            if (gamepad2.left_stick_y == 0 && isFirstZeroOfStick){ // If this is the first iteration of the loop with nobody moving the controller
                lastPosition = armMotor.getTargetPosition(); // Then set our brake to the current position
                isFirstZeroOfStick = false; // And tell all future iterations that we've handled the brake position.

            } else if (gamepad2.left_stick_y == 0){ // This part of the same selection block, if the first if statement is true, this will not execute, if nobody is trying to move, AND (this is NOT the first loop (implied))
                telemetry.addData("Arm locked!", ""); // Then we just tell people that the brake is engaged

            } else if (gamepad1.left_stick_y != 0){ // If someone is trying to move the arm, by moving the stick,
                isFirstZeroOfStick = true; // Then we set this back to true so the next time the stick is zero, it re-engages the brake
                lastPosition = 0; // And then we disengage the brake so that it doesn't counteract our movements

            }

            if (loopIterations%5/* The % operator tells you the remainder, if the first value is divisible by the second value, it returns zero, so this will only be true every x loops*/ == 0) { // Here we have the polling rate issue, the loop runs several hundred times per ms, and it checks values per loop, this means that by adding the values from the controller together, every iteration, we are going to get a very large value, very fast. This is resolved by not adding the values every iteration but every x iterations
                targetAddition = (int) ((gamepad2.left_stick_y) * COUNTS_PER_CM) + targetAddition; // by adding it to itself, you can give it high values regardless of motor position
            }

            armMotor.setTargetPosition(lastPosition+targetAddition); // Sets the position that the arm wants to go to




            // Send calculated power to wheels, and tell the arm to use full power when moving, this can be modified later to prevent overshoots, but its fine for now
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            armMotor.setPower(1);


            // Show a bunch of variables for debugging
            telemetry.addData("TargetPos: ", armMotor.getTargetPosition());
            telemetry.addData("CurrentPos: ", armMotor.getCurrentPosition());
            telemetry.addData("leftsticky ", gamepad2.left_stick_y);
            telemetry.addData("countspercm", COUNTS_PER_CM);
            telemetry.addData("targetAddition: ", targetAddition);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);                                                                                                                                                                                                                                                                                                                                                                                                        // Joe balls lmao
            telemetry.addData("Current mutliplier: ", powerMultiplier);
            telemetry.update();

            // This is an increment, it just adds one to the variable.
            loopIterations++;
        }
    }
}
