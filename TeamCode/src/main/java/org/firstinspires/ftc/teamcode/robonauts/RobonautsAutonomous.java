package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.StringUtils;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class RobonautsAutonomous extends LinearOpMode {
    String robotState = "START";
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        //----------------------------------------
        long startTimeInMilli = System.currentTimeMillis();

        Pose2d beginPose = new Pose2d(-36, -60, 0);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, beginPose);
        Drive drive = new Drive(mecanumDrive, hardwareMap, beginPose);
        Context context = new Context();
        ClawArm clawArm = new ClawArm(hardwareMap, telemetry, -300, context);
        ClawMain clawMain = new ClawMain(hardwareMap, startTimeInMilli, context);
        PixelDrop pixelDrop = new PixelDrop(hardwareMap, context);

        Actions.runBlocking(new SequentialAction(
                drive.strafeTo(-24,-50),
                pixelDrop.release(),
                drive.strafeTo(-36,-60),
                drive.turn(Math.toRadians(-85)),
                drive.toX(60)

                //drive.strafeTo(8,14 ),
                //drive.strafeTo(60,14 ),
                /*new ParallelAction(
                clawArm.extendToDropAtSpike(),
                        clawMain.release())*/

        ));

        //----------------------------------------
    /*--------------------------------un comment to go back
        ClawArm clawArm = new ClawArm(hardwareMap, telemetry, -300, "INIT");
        ClawMain clawMain = new ClawMain(hardwareMap);
        long startTimeInMilli = System.currentTimeMillis();
        MecanumDrive drive = null;
        RobotMovement robotMovement = new RobotMovement(hardwareMap, telemetry);
        //drive = robotMovement.moveToSpike(24,0);
        robotMovement.moveToSpikeLeft(20, 14);


        robotState = "REACHED_SPIKE";

        while (opModeIsActive()) {
            //Use camera to detect robot to move to
            //use api to move the robot to desired location,
            // extend the arm
            // angle the claw
            // release claw
            double power = clawArm.moveTo(-300);
            clawArm.setPower(power);
            //robotState = "ARM_EXTEND_1";
            telemetry.addData("giving constant power", power);

            telemetry.addData("Arm Motor current Position", clawArm.getCurrentPosition());

            telemetry.addData("Time difference", System.currentTimeMillis() - startTimeInMilli);
            if (System.currentTimeMillis() - startTimeInMilli >= 4000) {
                clawMain.setPower(0);
                robotState = "CLAW_EXTEND_1";
            } else  {
                clawMain.setPower(1);
            }
            telemetry.addData("ROBOT state:", robotState);

            //clawArm.moveTo(-350);
            if (robotState.equalsIgnoreCase("CLAW_EXTEND_1")) {
                robotState = "CLAW_RELEASED";
                telemetry.addData("ROBOT state:", "RELEASING.....");
                clawMain.releaseClaw();
            }
            telemetry.addData("ROBOT state:", robotState);

            telemetry.addData("ROBOT state:", robotState);

            telemetry.addData("Right Claw current Position", clawMain.getRightClawPosition());
            telemetry.addData("Left Claw  current Position", clawMain.getLeftClawPosition());

            telemetry.addData("Main current Position", clawMain.getMainClawPosition());

            telemetry.update();

        }
        */

    }
}
