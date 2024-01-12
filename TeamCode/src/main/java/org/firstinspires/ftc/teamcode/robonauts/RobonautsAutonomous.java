package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
        ClawArm clawArm = new ClawArm(hardwareMap, telemetry);
        ClawMain clawMain = new ClawMain(hardwareMap);
        long startTimeInMilli = System.currentTimeMillis();
        MecanumDrive drive = null;
        RobotMovement robotMovement = new RobotMovement(hardwareMap, telemetry);
        drive = robotMovement.moveToSpike(24, 0);

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

            if (robotState.equalsIgnoreCase("CLAW_RELEASED")) {
                robotState = "MOVING_TO_BB";
                //drive = robotMovement.moveToX(24, 60);
            }
            telemetry.addData("ROBOT state:", robotState);

            telemetry.addData("Right Claw current Position", clawMain.getRightClawPosition());
            telemetry.addData("Left Claw  current Position", clawMain.getLeftClawPosition());

            telemetry.addData("Main current Position", clawMain.getMainClawPosition());

            telemetry.update();

        }
    }
}
