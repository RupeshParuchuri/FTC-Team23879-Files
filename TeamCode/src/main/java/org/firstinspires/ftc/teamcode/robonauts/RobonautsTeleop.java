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
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class RobonautsTeleop extends LinearOpMode {
    LinearSlides linearSlides = null;
    boolean prevButtonAState = false;
    boolean prevButtonBState = false;

    private int SLIDE_DEPOSIT_POS=-700;
    private int SLIDE_PICKUP_POS=-200;

    Servo droneLauncher = null;
    private boolean prevButtonXState;
    private boolean prevButtonYState;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        this.droneLauncher = hardwareMap.get(Servo.class,"launcher");
        droneLauncher.setPosition(0.5); //rupesh added for launcher

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24, -24, Math.PI/2));
        Context context = new Context();
        linearSlides = new LinearSlides(hardwareMap, telemetry, 0, context);
        ClawArm clawArm  = new ClawArm(hardwareMap, telemetry, 0, context);
        ;
        ClawMain clawMain = new ClawMain(hardwareMap, 4000, context);
        PixelDrop pixelDrop = new PixelDrop(hardwareMap, context);

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
        double prevArmPower = 0.0;
        double prevClawMainPower = 0.0;
        while (opModeIsActive()) {
            //telemetry.addData("gamepad1.left_stick_y", -gamepad1.left_stick_y);
            //clawMain.setPower(-gamepad1.left_stick_y);
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
           /* boolean buttonAPressed = gamepad2.a;
            boolean buttonBPressed = gamepad2.b;
            if (!prevButtonAState && buttonAPressed) {
                linearSlides.setTargetPosition(SLIDE_DEPOSIT_POS, 0.5);
                telemetry.addData("Linear Slide   position set to", SLIDE_DEPOSIT_POS);

            }
            if (!prevButtonBState && buttonBPressed)
            {
                linearSlides.setTargetPosition(SLIDE_PICKUP_POS, 0.5);
                telemetry.addData("Linear Slide   position set to", SLIDE_PICKUP_POS);

            }
            prevButtonAState = buttonAPressed;
            prevButtonBState = buttonBPressed;



            linearSlides.moveTo();

            telemetry.addData("SLIDE   current position ", linearSlides.getCurrentPosition());
            telemetry.update();
*/
//////////////////////////////end slider position//////////////////////////
            ////////////////////////begin arm handling//////////////////////

            double clawMainPower = gamepad2.right_stick_y;
            clawMainPower = Math.abs(clawMainPower) >=0 ? clawMainPower: 0.0;
            if (clawMainPower != 0.0) {
                clawMain.setPower(clawMainPower);
                telemetry.addData("CLAW Main  Power set to", clawMainPower);

                prevClawMainPower = clawMainPower;
            } else if (prevClawMainPower != 0.0) {
                clawMain.setPower(0.0);
                prevClawMainPower = 0.0;
            }

            double armPower = gamepad2.left_stick_y;
            armPower = Math.abs(armPower) >=0 ? armPower: 0.0;
            if (armPower != 0.0) {
                clawArm.setPower(armPower);
                telemetry.addData("CLAW ARM  Power set to", armPower);

                prevArmPower = armPower;
            } else if (prevArmPower != 0.0) {
                clawArm.setPower(0.0);
                prevArmPower = 0.0;
            } else {
                boolean buttonXPressed = gamepad2.x;
                boolean buttonYPressed = gamepad2.y;
                boolean buttonAPressed = gamepad2.a;

                if (!prevButtonXState && buttonXPressed) {
                    clawArm.setTargetPosition(300, 0.045);
                    //while (Math.abs(clawArm.getRobotState().getArmPosition()) < 300) {
                    telemetry.addData("CLAW ARM  position set to", 100);
                    //clawArm.moveTo();
                    //}

                }
                if (!prevButtonYState && buttonYPressed)
                {
                    clawArm.setTargetPosition(900, 0.045);
                    // while (Math.abs(clawArm.getRobotState().getArmPosition()) < 50) {
                    telemetry.addData("CLAW ARM  position set to", 700);

                    //clawArm.moveTo();
                    //}
                }
                if (!prevButtonAState && buttonAPressed)
                {
                    clawArm.setTargetPosition(150, 0.01);
                    // while (Math.abs(clawArm.getRobotState().getArmPosition()) < 50) {
                    telemetry.addData("CLAW ARM  position set to", 0);

                    //clawArm.moveTo();
                    //}
                }
                telemetry.addData("button X Pressed.......... ", buttonXPressed);
                telemetry.addData("button Y Pressed.......... ", buttonYPressed);

                telemetry.addData("CLAW ARM  current position ", clawArm.getCurrentPosition());

                prevButtonXState=buttonXPressed;
                prevButtonYState=buttonYPressed;
                clawArm.moveTo();
                telemetry.update();
            }
            //////

            //////////////// claw////

            boolean button1APressed = gamepad1.a;
            boolean button1BPressed = gamepad1.b;

         if (gamepad2.left_trigger == 1) {
                clawMain.grab();
            }
         if (gamepad2.right_trigger==1) {
                clawMain.releaseClaw();
            }
         if (gamepad1.left_trigger == 1) {
                pixelDrop.release();
            }

         if(gamepad1.right_trigger == 1){
             droneLauncher.setPosition(0);
         }


            //clawArm.moveTo(-400);
            //clawArm.moveTo(-50);
            //////////////////////end arm handling/////////////////////////////////
        }
    }
}
