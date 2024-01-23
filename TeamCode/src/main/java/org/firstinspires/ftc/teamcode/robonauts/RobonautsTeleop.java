package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class RobonautsTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

       LinearSlides linearSlides = new LinearSlides(hardwareMap, telemetry, 0, "hold");
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
            telemetry.update();
        }
    }
}
