package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.StringUtils;

@TeleOp
@Config
public class ServoTest extends LinearOpMode {
    public static double position=0.5;
    public static  String servoName="drop";

    public static int timeForClawMain=1000;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Servo clawDrop = hardwareMap.get(Servo.class,"clawDrop");
        CRServo clawMain = hardwareMap.get(CRServo.class,"clawMain");
        Servo clawLeft = hardwareMap.get(Servo.class,"clawLeft");
        Servo clawRight = hardwareMap.get(Servo.class,"clawRight");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Claw drop Position before:", clawDrop.getPosition());
        telemetry.addData("Claw left Position before:", clawLeft.getPosition());
        telemetry.addData("Claw right Position before:", clawRight.getPosition());
        telemetry.addData("Claw main Position before:", clawMain.getPower());

        clawLeft.setDirection(Servo.Direction.REVERSE);
        long timeInMilli = System.currentTimeMillis();

        while(opModeIsActive()) {
            if (servoName.equalsIgnoreCase("left")) {
                clawLeft.setPosition(position);
            } else if (servoName.equalsIgnoreCase("right")) {
                clawRight.setPosition(position);

            } else if (servoName.equalsIgnoreCase("drop")) {
                clawDrop.setPosition(position);

            } else if (servoName.equalsIgnoreCase("main")) {
                //clawMain.setDirection(Servo.Direction.REVERSE);
                clawMain.setPower(position);
            }

    ///claw right 0.5 is claw release position  and 1 is clawpickup position.
                //clawDrop.setPosition(position);
            //clawDrop.setPosition(position);

            telemetry.addData("Claw drop Position after:", clawDrop.getPosition());
            telemetry.addData("Claw left Position after:", clawLeft.getPosition());
            telemetry.addData("Claw right Position after:", clawRight.getPosition());
            telemetry.addData("Claw Main Position after:", clawMain.getPower());
            telemetry.addData("-gamepad1.left_stick_y:", -gamepad1.left_stick_y);


            telemetry.update();
        }
    }
}
