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
    Servo clawMainServo = null;
    Servo clawLeft = null;

    Servo clawRight = null;
    Servo clawDrop = null;

    long startTimeMillis;
    Context robotState;

    public ClawMain(HardwareMap hardwareMap, long startTimeMillis, Context robotState) {
        this.clawMainServo=hardwareMap.get(Servo.class, "clawMain");
        this.clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        this.clawRight = hardwareMap.get(Servo.class, "clawRight");
        this.clawDrop = hardwareMap.get(Servo.class,"clawDrop");
        clawDrop.resetDeviceConfigurationForOpMode();
        clawLeft.resetDeviceConfigurationForOpMode();
        clawRight.resetDeviceConfigurationForOpMode();
        this.startTimeMillis = startTimeMillis;
        this.robotState = robotState;
    }



    public void releaseClaw() {
        //clawRight.setDirection(Servo.Direction.REVERSE);
        clawLeft.setPosition(0.7);

        clawRight.setPosition(0.7);
    }

    public void grab() {
        clawLeft.setPosition(0);
        clawRight.setPosition(1);
    }
    public class ReleaseToDropAtSpikeAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            //if (System.currentTimeMillis() - startTimeMillis >= 4000) {
            clawMainServo.setPosition(1);
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            releaseClaw();

            robotState.setRobotState("CLAW_RELEASED");

            telemetryPacket.put("Time greater than 4000...Releasing...", System.currentTimeMillis());

            return true;


        }
    }
    public Action release() {
        return new ReleaseToDropAtSpikeAction();
    }

}