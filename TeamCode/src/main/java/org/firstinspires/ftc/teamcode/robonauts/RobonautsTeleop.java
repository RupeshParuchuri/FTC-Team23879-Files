package org.firstinspires.ftc.teamcode.robonauts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobonautsTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.b) {
                LinearSlides linearSlides = new LinearSlides(hardwareMap, telemetry, -2000, "hold");
                linearSlides.extendToDropAtSpike();
            }
            if (gamepad2.a) {
                {
                    LinearSlides linearSlides = new LinearSlides(hardwareMap, telemetry, -100, "hold");
                    linearSlides.extendToDropAtSpike();
                }
            }
            if (gamepad2.x) {
                ClawArm clawArm = new ClawArm(hardwareMap, telemetry, -500, null);
                clawArm.extendToDropAtSpike();
                ClawMain clawMain = new ClawMain(hardwareMap, 4000, null);
                clawMain.release();
            }
        }
    }
}
