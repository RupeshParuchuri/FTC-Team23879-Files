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

    public ClawMain(HardwareMap hardwareMap, long startTimeMillis, Context robotState) {
        this.clawMainServo=hardwareMap.get(CRServo.class, "clawMain");
        this.clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        this.clawRight = hardwareMap.get(Servo.class, "clawRight");
        this.clawDrop = hardwareMap.get(Servo.class,"clawDrop");
        clawDrop.resetDeviceConfigurationForOpMode();
        clawLeft.resetDeviceConfigurationForOpMode();
        clawRight.resetDeviceConfigurationForOpMode();
        this.startTimeMillis = startTimeMillis;
        this.robotState = robotState;
    }

    public void setPosition(double power, CRServo.Direction direction) {
        clawMainServo.setPower(power);
        clawMainServo.setDirection(direction);
    }

    public void releaseClaw() {
        //clawRight.setDirection(Servo.Direction.REVERSE);
        //clawLeft.setPosition(0.5);
        clawRight.setPosition(0.5);
    }

    public void grab() {
        //clawMainServo.setPosition(1.0);
        clawLeft.setPosition(0.5);
        clawRight.setPosition(1);
    }
    public void stop() {
        clawMainServo.setPower(0);
    }

    public void setPower(double power) {
        clawMainServo.setPower(power);
    }
    public void toPickUpPosition(long millisecs) {

        long startTime = System.currentTimeMillis();
        //clawMainServo.setDirection(DcMotorSimple.Direction.REVERSE);
        //while (System.currentTimeMillis() - startTime < millisecs) {
            // Your code to run goes here

            // Add a delay if needed (optional)
            try {
                clawMainServo.setPower(1);
                Thread.sleep(10); // Sleep for a short duration
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        //}

    }
    public void depositPixel() {
        //clawMainServo.setPosition(0.5);
        //releaseClaw();
    }

    public double getLeftClawPosition() {
        return clawLeft.getPosition();
    }

    public double getRightClawPosition() {
        return clawRight.getPosition();
    }

    public double getMainClawPosition() {
        //return clawMainServo.getPosition();
        return 0;
    }


    public class ReleaseToDropAtSpikeAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
            }
            if (System.currentTimeMillis() - startTimeMillis >= 4000) {
                setPower(0);
                releaseClaw();
                robotState.setRobotState("CLAW_RELEASED");
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                telemetryPacket.put("Time greater than 4000...Releasing...", System.currentTimeMillis());
                return false;
            } else  {
                telemetryPacket.put("Time less than 4000...rotation...", System.currentTimeMillis());

                setPower(1);
                return true;
            }

        }
    }
    public Action release() {
        return new ReleaseToDropAtSpikeAction();
    }

}
