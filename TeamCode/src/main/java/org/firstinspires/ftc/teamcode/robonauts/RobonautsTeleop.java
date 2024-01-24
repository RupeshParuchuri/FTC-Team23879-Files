package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class RobonautsTeleop extends LinearOpMode {
    LinearSlides linearSlides = null;
    boolean prevButtonAState = false;
    boolean prevButtonBState = false;

    private int SLIDE_DEPOSIT_POS=-2000;
    private int SLIDE_PICKUP_POS=-200;


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24, -24, Math.PI/2));
        linearSlides = new LinearSlides(hardwareMap, telemetry, 0, "hold");
        ClawArm clawArm  = new ClawArm(hardwareMap, telemetry, 0, null);
        ;
        ClawMain clawMain = new ClawMain(hardwareMap, 4000, null);
        /*
        while (opModeIsActive()) {
            if (gamepad2.b) {
                telemetry.addData("Game Pad2 B clicked....Slide going to -2000 position", "-2000");
                 linearSlides.setTargetPosition(-2000);
                Actions.runBlocking(linearSlides.extendToDropAtBoard());
            }
            if (gamepad2.a) {
                {
                    telemetry.addData("Game Pad2 B clicked....Slide going to -2000 position", "-100");
                    linearSlides.setTargetPosition(-500);
                    Actions.runBlocking(linearSlides.extendToDropAtBoard());
                }
            }

            double stick = gamepad2.left_stick_y;
            while (stick != 0 ){
               // clawArm.extendToDropAtSpike();
                telemetry.addData("amepad2.left_stick_y", stick);
                clawMain.setPower(stick);
                Actions.runBlocking(clawMain.release());
            }*/
        while (opModeIsActive()) {
            telemetry.addData("gamepad1.left_stick_y", -gamepad1.left_stick_y);
            clawMain.setPower(-gamepad1.left_stick_y);
            //Actions.runBlocking(clawMain.release());

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));


            telemetry.update();
    ///////////////////////////Begin slides control///////////////////////
            boolean buttonAPressed = gamepad2.a;
            boolean buttonBPressed = gamepad2.b;
            if (!prevButtonAState && buttonAPressed)
            {
                linearSlides.setTargetPosition(SLIDE_DEPOSIT_POS, 0.5);
                linearSlides.moveTo();

            }

            if (!prevButtonBState && buttonBPressed)
            {
                linearSlides.setTargetPosition(SLIDE_PICKUP_POS, 0.5);
                linearSlides.moveTo();
            }
//////////////////////////////end slider position//////////////////////////
            ////////////////////////begin arm handling//////////////////////
            double armpower = -gamepad2.right_stick_x;
            armpower = Math.abs(armpower) >= 0 ? armpower : 0.0;
            clawArm.setPower(armpower);
            clawArm.moveTo();
            //clawArm.moveTo(-400);
            //clawArm.moveTo(-50);
            //////////////////////end arm handling/////////////////////////////////
        }
    }
}
