package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robonauts.vision.ObjectDetection;

@Autonomous
public class B3RobonautsAutonomous extends LinearOpMode {
    String robotState = "START";
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ObjectDetection objectDetection = new ObjectDetection(hardwareMap, telemetry);

        waitForStart();
        long startTimeInMilli = System.currentTimeMillis();
        String spikeLocation = "right";
        Pose2d beginPose = Constants.B_2_BEGIN_POSE;
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, beginPose);
        Drive drive = new Drive(mecanumDrive, hardwareMap, beginPose);
        Context context = new Context();

        ClawArm clawArm = new ClawArm(hardwareMap, telemetry, -280, context);
        ClawMain clawMain = new ClawMain(hardwareMap, startTimeInMilli, context);
        PixelDrop pixelDrop = new PixelDrop(hardwareMap, context);


        LinearSlides linearSlides = new LinearSlides(hardwareMap, telemetry, -2000, "hold");
        Actions.runBlocking(

                org.firstinspires.ftc.teamcode.robonauts.Actions.releasePixelAtBoardAction(mecanumDrive, "center", pixelDrop, linearSlides, clawArm, clawMain)
        );

        while (opModeIsActive()) {
            sleep(20);

            Double[] xAxis = objectDetection.getObjectDetectionsX();

            telemetry.update();
            telemetry.addData("Object coordinates", xAxis.length);

            if (System.currentTimeMillis() - startTimeInMilli > 2000 || xAxis.length > 0) {
                if (xAxis.length > 0) {
                    double xPos = xAxis[0];
                    if (xPos < 300 ) {
                        spikeLocation = "left";
                    } else if (xPos > 300 && xPos < 500)  {
                        spikeLocation = "center";
                    }
                }
                break;
            }

        }
        //----------------------------------------
        telemetry.addData("Prop location coordinates", spikeLocation);
        telemetry.update();
        //sleep(7000);
       // Pose2d beginPose = new Pose2d(-36, -60, Math.PI/2);

        //R1
        //Actions.runBlocking(org.firstinspires.ftc.teamcode.robonauts.Actions.get_R_1_ACTION(mecanumDrive, "center"));
        //R4
        //Actions.runBlocking(org.firstinspires.ftc.teamcode.robonauts.Actions.get_R_4_ACTION(mecanumDrive, "center"));

        //Actions.runBlocking(org.firstinspires.ftc.teamcode.robonauts.Actions.get_B_2_ACTION(mecanumDrive, "center", pixelDrop, linearSlides, clawArm, clawMain));
        //Actions.runBlocking(org.firstinspires.ftc.teamcode.robonauts.Actions.get_B_3_ACTION(mecanumDrive, "center"));

        //R4
        /*
        Actions.runBlocking(mecanumDrive.actionBuilder(beginPose)
                //.strafeTo(new Vector2d(-48,-36))
                .strafeTo(Constants.R_4_STRAFE_RELEASE_PIXEL_LEFT)
                .afterDisp(10, pixelDrop.release())
                //.strafeTo(new Vector2d(-36,-60))
                .strafeTo(Constants.R_4_STRAFE_BACK)
                //.strafeTo(new Vector2d(12,-60))
                                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(24,-36,0), Math.PI/2)
                .splineToLinearHeading(Constants.R_4_SPLINE_BB, Constants.R_4_HEADING_BB)
                //       .turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(24,-60))
                .build());

        //B3
        Actions.runBlocking(mecanumDrive.actionBuilder(beginPose)
                //.strafeTo(new Vector2d(-48,-36))
                .strafeTo(Constants.B_3_STRAFE_RELEASE_PIXEL_LEFT)
                .afterDisp(10, pixelDrop.release())
                //.strafeTo(new Vector2d(-36,-60))
                .strafeTo(Constants.B_3_STRAFE_BACK)
                //.strafeTo(new Vector2d(12,-60))
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(24,-36,0), Math.PI/2)
                .splineToLinearHeading(Constants.B_3_SPLINE_BB, Constants.B_3_HEADING_BB)
                //       .turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(24,-60))
                .build());


        Actions.runBlocking(mecanumDrive.actionBuilder(beginPose)
                //.strafeTo(new Vector2d(-48,-36))
                .strafeTo(Constants.B_2_STRAFE_RELEASE_PIXEL_LEFT)
                //.afterDisp(10, pixelDrop.release())
                //.strafeTo(new Vector2d(-36,-60))
                .strafeTo(Constants.B_2_STRAFE_BACK)
                .strafeTo(Constants.B_2_STRAFE_BB)
                //.strafeTo(new Vector2d(12,-60))
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(24,-36,0), Math.PI/2)
                .splineToLinearHeading(Constants.B_2_SPLINE_BB, Constants.B_2_HEADING_BB)
                //       .turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(24,-60))
                .build());

*/


/*


        TrajectoryActionBuilder actionBuilder = mecanumDrive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)));
        //TrajectoryActionBuilder secondActionBuilder = mecanumDrive.actionBuilder(mecanumDrive.pose);

        Actions.runBlocking(
                actionBuilder.setTangent(0).splineToConstantHeading(new Vector2d(24,24), Math.PI/2).build());


        Actions.runBlocking(new SequentialAction(
                actionBuilder.strafeTo(new Vector2d(-24, -50))
                .afterDisp(5, pixelDrop.release())
                        .build())
        );
        sleep(500);
        Actions.runBlocking(
                new SequentialAction(
                        mecanumDrive.actionBuilder(mecanumDrive.pose)
                                .setTangent(0)
                                .splineToLinearHeading( new Pose2d(-36, -55, -Math.PI/2), Math.PI/2).
                                build()));
        sleep(500);
       Actions.runBlocking(
                new SequentialAction(
                        mecanumDrive.actionBuilder(mecanumDrive.pose).strafeTo(new Vector2d(-36,-92)).
                                build()));*/
        /*
        Actions.runBlocking(
                new SequentialAction(
                        mecanumDrive.actionBuilder(new Pose2d(-36, -60, -Math.PI/2))
                                .setTangent(0)
                                .strafeTo(new Vector2d(-36, -100)).
                                build()));
                        //.strafeTo(new Vector2d(-30, -60))
                //.strafeTo(new Vector2d(30, -55)).build()));

*/


        /*
        actionBuilder.turn(Math.toRadians(90))
                        .lineToX(60);
        Actions.runBlocking(new SequentialAction(
                actionBuilder.build()
               // drive.strafeTo(-24,-50, beginPose),
                //pixelDrop.release(),
                //drive.strafeTo(-36,-60, beginPose),
                //actionBuilder.turn(Math.toRadians(-90)),
                //drive.strafeTo(30, -60, pose)
               //drive.toX(30, pose)

                //drive.strafeTo(8,14 ),
                //drive.strafeTo(60,14 ),
                *//*new ParallelAction(
                clawArm.extendToDropAtSpike(),
                        clawMain.release())*//*

        ));*/

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
