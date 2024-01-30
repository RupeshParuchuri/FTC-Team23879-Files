package org.firstinspires.ftc.teamcode.robonauts;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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






    public class ReleaseToDropAtSpikeAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
            }
           // releasePixel();
            clawDrop.setPosition(0.8);

            return false;



        }
    }

    public Action releasePixel() {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }
                clawDrop.setPosition(0.8);
                if (clawDrop.getPosition() == 0.8 ) {
                    return false;
                } else {
                    return true;
                }


            }
        };
    }
    public Action resetPixel() {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }
                clawDrop.setPosition(0.5);
                if (clawDrop.getPosition() == 0.5 ) {
                    return false;
                } else {
                    return true;
                }


            }
        };
    }

    public Action release() {
        return new ReleaseToDropAtSpikeAction();
    }

    public void reset() {
        clawDrop.setPosition(1);
    }

}
