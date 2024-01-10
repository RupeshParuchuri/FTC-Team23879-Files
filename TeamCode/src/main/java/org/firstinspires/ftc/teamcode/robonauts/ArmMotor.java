package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmMotor extends LinearOpMode {
    public CRServo clawMain;
    public Servo clawLeft;
    public Servo clawRight;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        clawMain = hardwareMap.get(CRServo.class, "clawMain");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawMain.resetDeviceConfigurationForOpMode();

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("clawMain", gamepad1.left_stick_y);
            telemetry.addData("clawLeft", gamepad1.right_stick_y);
            telemetry.addData("clawRight", gamepad1.right_stick_y);
            clawMain.setPower(1);
            telemetry.update();
            //clawLeft.setPosition(1);
        }
    }


    }




