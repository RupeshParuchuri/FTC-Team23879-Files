package org.firstinspires.ftc.teamcode.robonauts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;

@Autonomous
public class RobonautsAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        //Use camera to detect robot to move to
        //use api to move the robot to desired location,
        // extend the arm
        // angle the claw
        // release claw
        ClawArm clawArm = new ClawArm(hardwareMap);
        clawArm.moveTo(-350);
        telemetry.addData("Arm Motor current Position", clawArm.getCurrentPosition());
        ClawMain clawMain = new ClawMain(hardwareMap);
        clawMain.toPickUpPosition(3000);
        clawMain.releaseClaw();
        telemetry.addData("Right Claw current Position", clawMain.getRightClawPosition());
        telemetry.addData("Left Claw  current Position", clawMain.getLeftClawPosition());

        telemetry.addData("Main current Position", clawMain.getMainClawPosition());

        telemetry.update();

    }
}
