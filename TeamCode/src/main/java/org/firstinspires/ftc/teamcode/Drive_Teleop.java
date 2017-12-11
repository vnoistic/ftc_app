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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@TeleOp(name="New Drive Teleop", group="Practice Opmode")
public class Drive_Teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor clawMotor = null;

    // Toggle direction
    private boolean forward = true;

    // Vuforia stuff
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AbYJABr/////AAAAGZlNBbR1iULRqPgnEKN8rugaxEq7Wiqk7y8VS6WEH+FFCtSramQje2EJ9h+twE2FslQ6zkB/o9v1LfrOdNM7HmR+6uR2FcBoA3JnJKaJkhxsRN/sBQUvU3OEcGizbol1O2WS/nIO0TSFrFEnWcxN9o4HGcNj9M2z6nhEh78TkNYq+4zl3+mjreRe5xR+nnFpVCeY0qcG/4BqIYlTcSqTPYCY1BMy8tKDfD8te2M1Ur7qriIna4nGW5+kfE1/AJKbgmzmwNhESbuXf9m0AnfnJ60EWmXZSNJn9LexxBlBitHLXLTGdCctj6tINl3g135C8eoRspCoXYM8xk0u4vYPxFAGe1dbPY3MpUI1U33q6kjR";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

    }

    @Override
    public void init() { //Once, after init
        telemetry.addData("Status", "Init start.");

        // Load up Vuforia
        this.initVuforia();

        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        clawMotor = hardwareMap.get(DcMotor.class, "clawMotor");

        telemetry.addData("Debug", leftDrive);
        telemetry.addData("Debug", rightDrive);
        telemetry.addData("Debug", clawMotor);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Init end.");
    }

    @Override
    public void init_loop() { //Continuously, after init
        telemetry.addData("Status","Init looping.");
    }

    @Override
    public void start() {
        relicTrackables.activate(); // start tracking relics?
        runtime.reset();
    } //Once, after play

    @Override
    public void loop() { //Continuous, after play
        // Vuforia ID ---
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        }

        // Reverse Control --
        if (gamepad1.left_stick_button){
            forward = !forward;
            telemetry.addData("reverseControl", "%d", !forward);
        }

        // Change Control Mode --
        if (gamepad1.right_stick_button){
            controlMode = (controlMode + 1) % 3; // modes 0-2
            telemetry.addData("changeControl", "%d", controlMode);
        }

        // Drive Train ---
        if (controlMode == 0){ // Tank Mode controls
            double leftInput = gamepad1.left_stick_y;
            double rightInput = gamepad1.right_stick_y;

            if (!forward){ // Reverse controls
                leftInput = -leftInput;
                rightInput = -rightInput;
            }

            leftDrive.setPower(leftInput);
            rightDrive.setPower(rightInput);
            telemetry.addData("Input", "(tank) left (%.2f), right (%.2f)", leftInput, rightInput);
        }
        else{ // Arcade mode
            // TODO make sure orientation is correct

            double driveInput = gamepad1.right_stick_y;
            double turnInput = gamepad1.right_stick_x;

            if (controlMode == 2) { // Split arcade mode
                turnInput = gamepad1.left_stick_x;
            }

            double leftPower = Range.clip(driveInput + turnInput, -1.0, 1.0) ;
            double rightPower = Range.clip(driveInput - turnInput, -1.0, 1.0) ;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("Input", "(arcade) left (%.2f), right (%.2f)", leftPower, rightPower);
        }

        // Claw ---
//        if (gamepad1.left_bumper || gamepad2.left_bumper){
//            clawMotor.setPower(1.00);
//            telemetry.addData("Motors","Claw intake.");
//        }
//        else if (gamepad1.right_bumper || gamepad2.right_bumper){
//            clawMotor.setPower(-1.00);
//            telemetry.addData("Motors","Claw outtake.");
//        }
//        else{
//            clawMotor.setPower(0);
//            telemetry.addData("Motors","Claw inactive.");
//        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Runtime: " + runtime.toString());

        telemetry.update();
    }

    @Override
    public void stop() { //Once, after stop. Previously did nothing.
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        clawMotor.setPower(0);
    }

}
