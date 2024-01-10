package org.firstinspires.ftc.teamcode.robonauts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Timer;

public class ClawMain {
    CRServo clawMainServo = null;
    Servo clawLeft = null;

    Servo clawRight = null;


    public ClawMain(HardwareMap hardwareMap) {
        this.clawMainServo=hardwareMap.get(CRServo.class, "clawMain");
        this.clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        this.clawRight = hardwareMap.get(Servo.class, "clawRight");
    }

    public void setPosition(double power, CRServo.Direction direction) {
        clawMainServo.setPower(power);
        clawMainServo.setDirection(direction);
    }

    public void releaseClaw() {
        clawLeft.setPosition(0);
        clawRight.setPosition(0);
    }

    public void grab() {
        //clawMainServo.setPosition(1.0);
        clawLeft.setPosition(0.5);
        clawRight.setPosition(0.5);
    }

    public void toPickUpPosition(long millisecs) {

        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < millisecs) {
            // Your code to run goes here

            // Add a delay if needed (optional)
            try {
                clawMainServo.setPower(1);
                Thread.sleep(10); // Sleep for a short duration
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
    public void depositPixel() {
        //clawMainServo.setPosition(0.5);
        releaseClaw();
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
}
