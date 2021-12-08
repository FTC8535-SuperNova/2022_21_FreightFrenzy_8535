/* Copyright (c) 2019 FIRST. All rights reserved.
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


import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;


/**
 * This OpMode illustrates using the Vuforia localizer to determine positioning and orientation of
 * robot on the FTC field using a WEBCAM.  The code is structured as a LinearOpMode
 *
 * NOTE: If you are running on a Phone with a built-in camera, use the ConceptVuforiaFieldNavigation example instead of this one.
 * NOTE: It is possible to switch between multiple WebCams (eg: one for the left side and one for the right).
 *       For a related example of how to do this, see ConceptTensorFlowObjectDetectionSwitchableCameras
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * Finally, the location of the camera on the robot is used to determine the
 * robot's location and orientation on the field.
 *
 * To learn more about the FTC field coordinate model, see FTC_FieldCoordinateSystemDefinition.pdf in this folder
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="Vuforia Field Nav Webcam", group ="Concept")

public class ConceptVuforiaFieldNavigationWebcam extends LinearOpMode {


    private Vuforia vuforia = new Vuforia(false,"AQq/tAz/////AAABmQLdTKfIqUYXtldONwoyIpt7LuTPID2/eRq05VBghKTN/JlWD5muCZjHl0d0gYu9jlfUdMyvqJRVGPVlqDouTF1bcS9DMTB1LMFNPhHmQspaaqs2MkDFxDScBncQLegWEaJfm4oiuIwAK18SHeHqg8yNoH2LQZa/HoobNlZME5Xm/YpwK8oxv+1qEMezceJ03ANFTpbPDFFmD8rOGR79Ms6/p1O1i+S8ZKrujs9E5r9pHQrD7zCr8o2bxCha3PIFyX9SS4sxeHIX4/W9Bjd++0Tb3aU7MV607WwkgQvZKYPN27X/y3f6JyiCzQFwZMpbMRtSMczsAs3zPFA9C8ZDuq+d4bL023dshULTmnUeWVzT\n");

    public int hasError = 0;
    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime  = new ElapsedTime();

    private AutoDriveByGyro_Instantiated gyroDriving = new AutoDriveByGyro_Instantiated(hardwareMap);


    /** A method that uses cool math, vuforia, and the gyro to drive to a specific location relative to the field, not the robot.


     */
    private void vuforiaDriveToPos(double targetX, double targetZ, double power, double finalAngle) {

        vuforia.updateVuforia();
        double currentX = 1;
        double currentZ = 1;
        try {
            currentX = vuforia.translation.get(0);
            currentZ = vuforia.translation.get(2);

        } catch(Exception NullPointerException){
            RobotLog.addGlobalWarningMessage("Called vuforia.translation.get(int); without a visible target!");
            hasError = 1;

            return;
        }
        double xTranslation = currentX - targetX;
        double zTranslation = currentZ - targetZ;

        // Target distance is in inches
        double targetRange = Math.hypot(xTranslation, zTranslation);

        // target bearing is based on angle formed between the X axis and the target range line
        double targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));
        gyroDriving.gyroDrive(power, targetRange, targetBearing);

        if(finalAngle != -1) {
            gyroDriving.gyroTurn(power, targetBearing);


        }
    }


    @Override public void runOpMode()
    {
        vuforia.initVuforia();

        robot.init(hardwareMap);

        gyroDriving.init();

        waitForStart();
        runtime.reset();


        vuforiaDriveToPos(0,0,1,1);

    }
}