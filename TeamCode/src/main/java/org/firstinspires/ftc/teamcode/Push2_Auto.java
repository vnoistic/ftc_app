package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by timmy on 10/12/17.
 */
@Autonomous(name="x Push Autonomous Right", group="Practice Opmode")
public class Push2_Auto extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor clawMotor = null;

    // Vuforia stuff
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private long timeStamp;

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
        telemetry.addData("Status", "First time init.");

        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        clawMotor = hardwareMap.get(DcMotor.class, "clawMotor");

        telemetry.addData("Debug", leftDrive);
        telemetry.addData("Debug", rightDrive);
        telemetry.addData("Debug", clawMotor);

        this.initVuforia();

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Init end.");


    }

    @Override
    public void init_loop() { //Continuously, after init
        telemetry.addData("1", "   -- Ruse Robotics --");
        telemetry.addData("2", "Good Luck! Hope things don't break!");
        telemetry.addData("3", "- Full auto - read mark and push block");
    }

    @Override
    public void start() {
        telemetry.addData("Status","First time start.");
        runtime.reset();
        timeStamp = System.currentTimeMillis();
    } //Once, after play

    private RelicRecoveryVuMark column = RelicRecoveryVuMark.UNKNOWN;

    @Override
    public void loop() { //Continuous, after play

        // Vuforia ID ---
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        }

        if (System.currentTimeMillis() < timeStamp + 100) {
            // Try read the mark. Make sure bot is facing the block
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                column = vuMark;
            }
        } else if (System.currentTimeMillis() < timeStamp + 4000) {
            telemetry.addData("Status", "Move across a bit, for better aiming");
            // Move over a bit
            leftDrive.setPower(0.5);
            rightDrive.setPower(0.5);
        } else if (System.currentTimeMillis() < timeStamp + 6000) {
            // Turn to correct position. Fix power level to change
            telemetry.addData("Status", "Turn towards correct column");
            telemetry.addData("Column", "Centre (or not recognised)");
            leftDrive.setPower(-1);
            rightDrive.setPower(1);
        } else if (System.currentTimeMillis() < timeStamp + 5000) {
            telemetry.addData("Status", "Moving forward into spot");
            leftDrive.setPower(1);
            rightDrive.setPower(1);
        } else {
            telemetry.addData("Status", "Parking");
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

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
