package org.firstinspires.ftc.teamcode.robonauts;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelDrop {

    Servo clawDrop = null;

    Context robotState;

    public PixelDrop(HardwareMap hardwareMap, Context robotState) {

        this.clawDrop = hardwareMap.get(Servo.class,"clawDrop");
        clawDrop.resetDeviceConfigurationForOpMode();

        this.robotState = robotState;
    }



    public void releasePixel() {
        //clawDrop.setDirection(Servo.Direction.REVERSE);
        //clawLeft.setPosition(0.5);
        clawDrop.setPosition(1);

    }



    public class ReleaseToDropAtSpikeAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
            }
            releasePixel();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            if (clawDrop.getPosition() == 1) {
                clawDrop.setPosition(0);

                return false;
            } else {
                return true;
            }

        }
    }
    public Action release() {
        return new ReleaseToDropAtSpikeAction();
    }

}
