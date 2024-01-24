package org.firstinspires.ftc.teamcode.robonauts.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@TeleOp(name="Vision Test")
public class CameraTest extends LinearOpMode {

    private VisionPortal portal;
    private RedPropThreshold redPropThreshold;

    ColourMassDetectionProcessor colourMassDetectionProcessor;
    @Override
    public void runOpMode() throws InterruptedException {
        redPropThreshold = new RedPropThreshold();
        Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        double minArea = 100; // the minimum area for the detection to consider for your prop

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "c920-webcam"))
                .setCameraResolution(new Size(640, 480))
                //.setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(colourMassDetectionProcessor)
                .build();


        waitForStart();
        telemetry.addData("Prop Position", colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.update();

    }
}
