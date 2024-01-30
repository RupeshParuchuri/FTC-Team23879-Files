package org.firstinspires.ftc.teamcode.robonauts;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robonauts.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.robonauts.vision.EnhancedColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.robonauts.vision.ObjectDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class R1RobonautsAutonomous extends LinearOpMode {
    String robotState = "START";

    ColourMassDetectionProcessor colourMassDetectionProcessor;
    VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection



        double minArea = 100; // the minimum area for the detection to consider for your prop


        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "c920-webcam"))
                .setCameraResolution(new Size(640, 480))
                //.setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(colourMassDetectionProcessor)
                .build();


        waitForStart();
        if (opModeIsActive()) {
        long startTimeInMilli = System.currentTimeMillis();
        ColourMassDetectionProcessor.PropPositions propPositions = colourMassDetectionProcessor.getRecordedPropPosition();

        String spikeLocation = propPositions.name();
        telemetry.addData("Prop location coordinates", spikeLocation);
        telemetry.update();
        if (propPositions.name().equalsIgnoreCase(ColourMassDetectionProcessor.PropPositions.UNFOUND.name())) {
            spikeLocation = "RIGHT";
        }
        Pose2d beginPose = Constants.R_1_BEGIN_POSE;
        Pose2d beginPoseForRelease = new Pose2d( new Vector2d(
                48, -12), 0);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, beginPose);

        MecanumDrive mecanumDrive1 = new MecanumDrive(hardwareMap, beginPoseForRelease);

            Context context = new Context();
            context.setClawMainSource("Auto");
        ClawArm clawArm = new ClawArm(hardwareMap, telemetry, -300, context);
        ClawMain clawMain = new ClawMain(hardwareMap, startTimeInMilli, context);
        PixelDrop pixelDrop = new PixelDrop(hardwareMap, context);


        if (spikeLocation.equalsIgnoreCase("LEFT")) {
            Actions.runBlocking(
                    new SequentialAction(

                            org.firstinspires.ftc.teamcode.robonauts.Actions.get_R1_DRIVE_TO_LEFT_PIXEL(mecanumDrive, pixelDrop, clawArm, clawMain),
                    org.firstinspires.ftc.teamcode.robonauts.Actions.dropPurplePixel(mecanumDrive, pixelDrop),
                    org.firstinspires.ftc.teamcode.robonauts.Actions.get_R1_LEFT_TO_BB(mecanumDrive, pixelDrop, clawArm, clawMain),
                    org.firstinspires.ftc.teamcode.robonauts.Actions.dropPurplePixel(mecanumDrive, pixelDrop),
                    org.firstinspires.ftc.teamcode.robonauts.Actions.RedPark(mecanumDrive, pixelDrop, clawArm, clawMain)
                    )
                    );
        } else if (spikeLocation.equalsIgnoreCase("MIDDLE")) {
            Actions.runBlocking(
                    new SequentialAction(
                            org.firstinspires.ftc.teamcode.robonauts.Actions.get_R1_DRIVE_TO_CENTER_PIXEL(mecanumDrive, pixelDrop, clawArm, clawMain),
                            org.firstinspires.ftc.teamcode.robonauts.Actions.dropPurplePixel(mecanumDrive, pixelDrop),
                            org.firstinspires.ftc.teamcode.robonauts.Actions.get_R1_CENTER_TO_BB(mecanumDrive, pixelDrop, clawArm, clawMain),
                            org.firstinspires.ftc.teamcode.robonauts.Actions.releaseYellowPixel(mecanumDrive, clawMain),
                            org.firstinspires.ftc.teamcode.robonauts.Actions.RedPark(mecanumDrive, pixelDrop, clawArm, clawMain)
                            )
            );

        } else {
            Actions.runBlocking(
                    new SequentialAction(

                            org.firstinspires.ftc.teamcode.robonauts.Actions.get_R1_DRIVE_TO_RIGHT_PIXEL(mecanumDrive, pixelDrop, clawArm, clawMain),
                    org.firstinspires.ftc.teamcode.robonauts.Actions.dropPurplePixel(mecanumDrive, pixelDrop),
                    org.firstinspires.ftc.teamcode.robonauts.Actions.get_R1_RIGHT_TO_BB(mecanumDrive, pixelDrop, clawArm, clawMain),
                    org.firstinspires.ftc.teamcode.robonauts.Actions.releaseYellowPixel(mecanumDrive, clawMain),
                    org.firstinspires.ftc.teamcode.robonauts.Actions.RedPark(mecanumDrive, pixelDrop, clawArm, clawMain)
                    )
                    );

        }

        telemetry.addData("Prop location coordinates", spikeLocation);

        telemetry.update();


        }


    }
}
