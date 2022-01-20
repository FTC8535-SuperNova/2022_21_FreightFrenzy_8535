package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@Autonomous(name="DO NOT USE UNTIL FUTHUR NOTICE", group="Linear Opmode")

public class BotAutonomousArmMotorTestBlue extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.init(hardwareMap);

        robot.leftDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.rightDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.armMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        robot.armMotor.setTargetPosition(0);
        Thread.sleep(500);

        robot.armServo.setPosition(1);
        Thread.sleep(500);

        robot.leftDrive.setPower(-0.75);
        robot.rightDrive.setPower(-0.75);
        Thread.sleep(250);

        robot.leftDrive.setPower(-0.2);
        robot.rightDrive.setPower(-0.2);
        Thread.sleep(250);

        robot.armMotor.setTargetPosition(1400);
        Thread.sleep(500);

        robot.armServo.setPosition(0);
        Thread.sleep(400);

        robot.armMotor.setTargetPosition(0);


        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.armMotor.setTargetPosition(0);

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}

// this is for blue side by Utsav (-o-)