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


/**
 * This is a basic teleop program used for driver testing
 *
 * @author Lars, Thomas Carver
 * @version 1.2.2
 *
 *
 */
@TeleOp(name="Basic: Linear OpMode")

public class BasicTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo clawServo = null;
    private DcMotor armMotor = null;
    private CRServo duckServo = null;





    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        armMotor = hardwareMap.get(DcMotor.class,"arm_motor");
        duckServo = hardwareMap.get(CRServo.class, "duck_servo");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);



        boolean servoIsRunning = false;
        double powerMultiplier = 0.75;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel and the arm to save power level for telemetry
            double leftPower;
            double rightPower;
            double armPower;




            // This is an E-Stop or Emergency stop, there is one on the phone, but this one has additional clauses that make it specific to the program, additionally it does not require the operator to put down the controller first
            if (gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1 || gamepad1.left_trigger == 1 && gamepad2.right_trigger == 1){
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
                clawServo.setPosition(0.90);

            } else if (gamepad2.b){
                clawServo.setPosition(0.30);

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


                //A is for closing the claw
            if (gamepad1.a){ // This is what I call a "virtual gearshift" it just allows you to change the speed of the robot on the fly for precision movements
                powerMultiplier = 1; // 1 is full speed, you get the idea


                //B is for opening the claw
            } else if (gamepad1.b){
                powerMultiplier = 0.60;

            } else if (gamepad1.y){
                powerMultiplier = 0.5;

            } else if (gamepad1.x){
                powerMultiplier = 0.2;

            } // end v-gearshift  if/else if/else if/else if



            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y; // These are reversed so that the movements on the controller correspond to actual robot movements
            double turn  =  -gamepad1.left_stick_x;

            leftPower    = Range.clip((drive + turn)*powerMultiplier, -1.0, 1.0) ; //Range.clip makes sure that a value fits withing the min and max values specified, you don;t need to understand it, when you set power, just use it.
            rightPower   = Range.clip((drive - turn)*powerMultiplier, -1.0, 1.0) ; // Heres where the power multiplier comes in, it is just the power that was going to be sent to the wheels, multiplied by the number.
            armPower    = Range.clip(gamepad2.right_stick_y, -1, 1); // Because no math is performed on the arm, the controller is referenced directlyw


            // Send calculated power to wheels and the arm
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            armMotor.setPower(armPower);

            // Show data that could be helpful for driving or debug
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Current mutliplier: ", powerMultiplier);
            telemetry.update();
        } // End opModeIsActive while
    } // End opMode(void) method, returns void
} // End class
