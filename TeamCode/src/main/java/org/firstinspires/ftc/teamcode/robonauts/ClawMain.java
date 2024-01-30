package org.firstinspires.ftc.teamcode.robonauts;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Timer;

public class ClawMain {
    CRServo clawMainServo = null;
    Servo clawLeft = null;

    Servo clawRight = null;
    Servo clawDrop = null;

    long startTimeMillis;
    Context robotState;

    private double power;

    private int count = 0;

    public ClawMain(HardwareMap hardwareMap, long startTimeMillis, Context robotState) {
        this.clawMainServo=hardwareMap.get(CRServo.class, "clawMain");
        this.clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        this.clawRight = hardwareMap.get(Servo.class, "clawRight");
        this.clawDrop = hardwareMap.get(Servo.class,"clawDrop");
        this.startTimeMillis = startTimeMillis;
        this.robotState = robotState;
    }



    public void releaseClaw() {
        clawLeft.setPosition(0.2);
        clawRight.setPosition(0.8);
    }

    public void grab() {
        clawLeft.setPosition(0);
        clawRight.setPosition(1);
    }

    public void setTimeToOperate(long time) {
        this.startTimeMillis = time;
    }

    public void setPower(double power) {
        this.clawMainServo.setPower(power);
    }

    public void setRobotState() {
        this.robotState.setRobotState("MAINCLAWEXTENDED");
    }

    public class ReleaseToDropAtSpikeAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
            }
            telemetryPacket.put("Current State in ClawMain...",  robotState.getRobotState() + System.currentTimeMillis());


            if (clawLeft.getPosition() == 0.2 || clawRight.getPosition() == 0.8) {
                return false;
            } else {
                return true;
            }

        }
    }



    public class PickUpClawAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
            }

            //clawMainServo.setPosition(1);
            grab();
            if (clawLeft.getPosition() == 0 && clawRight.getPosition() == 1) {
                return false;
            } else {
                return true;
            }

        }
    }


    public Action releaseClawAction() {
        //clawMainServo.setPosition(position);
        return new ReleaseToDropAtSpikeAction();

    }

}
