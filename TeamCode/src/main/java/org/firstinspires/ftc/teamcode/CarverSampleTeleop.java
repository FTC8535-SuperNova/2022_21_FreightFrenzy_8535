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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


/** This is sample code for an encoder driven arm motor, for the herndon HS scrimmage
 *
 * @version 1.3.2
 * @author Thomas Carver
 */
@TeleOp(name="Sample Teleop for scrim", group="Linear Opmode")
public class CarverSampleTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime =   new ElapsedTime();
    RobotHardware robot =           new RobotHardware();
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
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // The motor encoders need to be zeroed every time you initialize
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for it to finish resetting
        Thread.sleep(500);

        // You cannot be in DcMotor.RunMode.RUN_TO_POSITION without having a target pos
        robot.armMotor.setTargetPosition(0);


        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // All setModes must be done after hardware init
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Some variables for use in the loop, these have set values at the start and as such they are outside of the loop
        double powerMultiplier = 0.75;
        double targetAddition = 0; // Variable for math
        boolean servoIsRunning = false; // Variable for toggleable servo


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // This is an E-Stop or Emergency stop, there is one on the phone, but this one has additional clauses that make it specific to the program, additionally it does not require the operator to put down the controller first
            if (gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1|| gamepad1.left_trigger == 1 && gamepad1.right_trigger == 1){
                RobotLog.addGlobalWarningMessage("E-Stop triggered!"); // This is how you declare warning messages that stick around after program stop

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.armMotor.setPower(0);
                robot.duckServo.setPower(0);// i think that this will stop it from moving at the stat but i am not sure i will change it if it is wrong

                stop(); // The stop method is a part of linearOpMode, does some other things, and can delay stops for up to 100 days if necessary

                ExecutorService executor = Executors.newSingleThreadExecutor();
                executor.shutdownNow(); // Okay, maybe this is a bit overkill, but if the method gets this far, this will force the executor service to terminate the thread

            } //

            // This is to control the servo for the claw
            if (gamepad2.a) {
                robot.armServo.setPosition(0.30);

            } else if (gamepad2.b){
                robot.armServo.setPosition(0.80);

            } // end claw servo if/else if



            // This is to control the CR (continuous rotation) servo for the ducks
            if (gamepad2.x){
                robot.duckServo.setPower(1); // CR servos use setPower rather than setPosition



            } else if (gamepad2.y){
                robot.duckServo.setPower(-1);



            } else {
                robot.duckServo.setPower(0);
            } // end duck servo control else if



            if (gamepad1.a){ // This is what I call a "virtual gearshift" it just allows you to change the speed of the robot on the fly for precision movements
                powerMultiplier = 1; // 1 is full speed, you get the idea

            } else if (gamepad1.b){
                powerMultiplier = 0.5;

            } else if (gamepad1.y){
                powerMultiplier = 0.2;

            } // end v-gearshift  if/else if/else if/else if



            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;

            leftPower    = Range.clip((drive + turn)*powerMultiplier, -1.0, 1.0);
            rightPower   = Range.clip((drive - turn)*powerMultiplier, -1.0, 1.0);


            targetAddition += ((-gamepad2.left_stick_y) * COUNTS_PER_CM * 1.25); // by adding it to itself, you can give it high values regardless of motor position
            if (targetAddition < 0) {
                targetAddition = 0;
            }
            if (targetAddition > 1400) {
                targetAddition = 1400;
            }
            robot.armMotor.setTargetPosition((int)targetAddition); // Sets the position that the arm wants to go to


            // Send calculated power to wheels
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

            // Tell the arm to move slower the closer it is to target
            double armPower = (robot.armMotor.getTargetPosition()-robot.armMotor.getCurrentPosition())/20.0;
            robot.armMotor.setPower(armPower);


            // Show a bunch of variables for debugging
            telemetry.addData("TargetPos: ", robot.armMotor.getTargetPosition());
            telemetry.addData("CurrentPos: ", robot.armMotor.getCurrentPosition());
            telemetry.addData("CurrentPower: ", armPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);                                                                                                                                                                                                                                                                                                                                                                                                        // Joe balls lmao
            telemetry.addData("Current mutliplier: ", powerMultiplier);
            telemetry.update();
        }
    }
}
