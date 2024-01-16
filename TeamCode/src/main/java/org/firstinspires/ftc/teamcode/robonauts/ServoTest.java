package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTest extends LinearOpMode {
    public static double position=0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Servo clawDrop = hardwareMap.get(Servo.class,"clawDrop");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Position before:", clawDrop.getPosition());
        while(opModeIsActive()) {
            clawDrop.setPosition(position);
            telemetry.addData("Position before:", clawDrop.getPosition());
            telemetry.update();
        }
    }
}
