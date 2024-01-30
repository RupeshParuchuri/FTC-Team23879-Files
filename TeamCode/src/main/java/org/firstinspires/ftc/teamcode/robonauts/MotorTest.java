package org.firstinspires.ftc.teamcode.robonauts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MotorTest extends LinearOpMode {
    DcMotorEx dcMotorEx;
    @Override
    public void runOpMode() throws InterruptedException {
        dcMotorEx = hardwareMap.get(DcMotorEx.class, "par0");

        waitForStart();
        while (opModeIsActive()) {
            dcMotorEx.setPower(0.5);
        }
    }
}
