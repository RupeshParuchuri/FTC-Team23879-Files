package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.Time;

import java.util.TimerTask;

@TeleOp
@Config
public class ClawMainTuning extends OpMode {
    CRServo clawMainServo = null;
    public static double targePower = 0;

    public static int direction = 0;

    public static int timeInMilli = 0;

    Timing.Timer mainClawTimer = null;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mainClawTimer = new Timing.Timer(timeInMilli);
        this.clawMainServo=hardwareMap.get(CRServo.class, "clawMain");
    }

    @Override
    public void start() {
        mainClawTimer.start();
    }
    @Override
    public void loop() {
        telemetry.addData("Power Before", targePower);
        telemetry.addData("Position", clawMainServo.getDirection());

        if(mainClawTimer.isTimerOn()) {
            telemetry.addData("Timer On Power 1 ", clawMainServo.getDirection());
            clawMainServo.setPower(targePower);

        } else {
            clawMainServo.setPower(0);
            telemetry.addData("Timer Off Power to 0", clawMainServo.getDirection());

        }

        telemetry.addData("Time", clawMainServo.getDirection());

        //clawMainServo.setPosition(targetMotorPosition);
        if (direction == 0) {
            clawMainServo.setDirection(CRServo.Direction.REVERSE);
        } else {
            clawMainServo.setDirection(CRServo.Direction.FORWARD);

        }
        telemetry.update();
    }
}
