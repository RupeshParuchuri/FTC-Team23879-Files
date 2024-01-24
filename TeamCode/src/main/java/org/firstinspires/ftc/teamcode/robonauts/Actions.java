package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robonauts.vision.ColourMassDetectionProcessor;
import org.opencv.core.Mat;

public class Actions {
    public static Action get_R_1_ACTION (MecanumDrive mecanumDrive, String direction, PixelDrop pixelDrop,
                                         LinearSlides linearSlides, ClawArm clawArm, ClawMain clawMain) {

        Vector2d spikeLocation = Constants.R_1_STRAFE_RELEASE_PIXEL_CENTER;
        if (direction.equalsIgnoreCase(ColourMassDetectionProcessor.PropPositions.LEFT.name())) {
            spikeLocation = Constants.R_1_STRAFE_RELEASE_PIXEL_LEFT;
        } else if (direction.equalsIgnoreCase(ColourMassDetectionProcessor.PropPositions.MIDDLE.name())) {
            spikeLocation = Constants.R_1_STRAFE_RELEASE_PIXEL_CENTER;
        } else {
            spikeLocation = Constants.R_1_STRAFE_RELEASE_PIXEL_RIGHT;
        }

        return mecanumDrive.actionBuilder(Constants.R_1_BEGIN_POSE)
                .strafeTo(spikeLocation)
                .afterDisp(0.1, pixelDrop.release())
                .afterTime(1, () -> {
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })

                .strafeTo(new Vector2d(-36, -48))
                .strafeTo(new Vector2d(-36, -12))

                //.strafeTo(new Vector2d(-36, -36))
                //.splineToLinearHeading(new Pose2d( new Vector2d(-36,-24), 0), Constants.R_1_HEADING_BB)
                .turn(Math.toRadians(-90))

                .strafeTo(new Vector2d(-12, -12))
                //.strafeTo(Constants.R_1_STRAFE_BB)
                //.splineToLinearHeading(Constants.R_1_SPLINE_BB, Constants.R_1_HEADING_BB)
                //.strafeTo(new Vector2d(46, -12))
                //.strafeTo(Constants.R_PARK_BB)
                //.splineToLinearHeading(new Pose2d( Constants.R_1_STRAFE_BACK, 0), Constants.R_1_HEADING_BB)
                //.splineToLinearHeading(new Pose2d( Constants.R_1_STRAFE_BACK, 0), Constants.R_1_HEADING_BB)

                //.turn(Math.toRadians(180))
                //.strafeTo(Constants.R_1_STRAFE_BACK)
                //.strafeTo(Constants.R_1_STRAFE_BB)
                //.splineToLinearHeading(Constants.R_1_SPLINE_BB, Constants.R_1_HEADING_BB)
                //.afterTime(0.2, releasePixelAtBoardAction(mecanumDrive, direction, pixelDrop, linearSlides, clawArm, clawMain))
                //.strafeTo(Constants.B_BEFORE_PARK_BB)
                //.strafeTo(Constants.R_PARK_BB)
                //.afterTime(0.2, () -> linearSlides.setTargetPosition(-150))
                //.afterTime(0.2, releasePixelAtBoardAction(mecanumDrive, direction, pixelDrop, linearSlides, clawArm, clawMain))
                .build();

    }
    public static Action get_R_4_ACTION (MecanumDrive mecanumDrive, String direction) {
        Vector2d spikeLocation = Constants.R_4_STRAFE_RELEASE_PIXEL_CENTER;
        if (direction.equalsIgnoreCase("left")) {
            spikeLocation = Constants.R_4_STRAFE_RELEASE_PIXEL_LEFT;
        } else if (direction.equalsIgnoreCase("right")) {
            spikeLocation = Constants.R_4_STRAFE_RELEASE_PIXEL_RIGHT;
        }
        return mecanumDrive.actionBuilder(Constants.R_4_BEGIN_POSE )
                //.strafeTo(new Vector2d(-48,-36))
                .strafeTo(spikeLocation)
                //.afterDisp(10, pixelDrop.release())
                //.strafeTo(new Vector2d(-36,-60))
                //.strafeTo(Constants.R_4_STRAFE_BACK)
                //.strafeTo(new Vector2d(12,-60))
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(24,-36,0), Math.PI/2)
                .splineToLinearHeading(Constants.R_4_SPLINE_BB, Constants.R_4_HEADING_BB)
                //       .turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(24,-60))
                //.strafeTo(Constants.R_PARK_BB)

                .build();
    }

    public static Action get_B_3_ACTION (MecanumDrive mecanumDrive, String direction) {
        Vector2d spikeLocation = Constants.B_3_STRAFE_RELEASE_PIXEL_CENTER;
        if (direction.equalsIgnoreCase("left")) {
            spikeLocation = Constants.B_3_STRAFE_RELEASE_PIXEL_LEFT;
        } else if (direction.equalsIgnoreCase("right")) {
            spikeLocation = Constants.B_3_STRAFE_RELEASE_PIXEL_RIGHT;
        }
        return mecanumDrive.actionBuilder(Constants.B_3_BEGIN_POSE)
                //.strafeTo(new Vector2d(-48,-36))
                .strafeTo(spikeLocation)
                //.afterDisp(10, pixelDrop.release())
                //.strafeTo(new Vector2d(-36,-60))
                //.strafeTo(Constants.B_3_STRAFE_BACK)
                //.strafeTo(new Vector2d(12,-60))
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(24,-36,0), Math.PI/2)
                .splineToLinearHeading(Constants.B_3_SPLINE_BB, Constants.B_3_HEADING_BB)

                //       .turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(24,-60))
                .strafeTo(Constants.B_BEFORE_PARK_BB)
                .strafeTo(Constants.B_PARK_BB)

                .build();
    }

    public static Action get_B_2_ACTION (MecanumDrive mecanumDrive, String direction, PixelDrop pixelDrop,
                                         LinearSlides linearSlides, ClawArm clawArm, ClawMain clawMain) {
        Context context = new Context();

        Vector2d spikeLocation = Constants.B_2_STRAFE_RELEASE_PIXEL_CENTER;
        if (direction.equalsIgnoreCase("left")) {
            spikeLocation = Constants.B_2_STRAFE_RELEASE_PIXEL_LEFT;
        } else if (direction.equalsIgnoreCase("right")) {
            spikeLocation = Constants.B_2_STRAFE_RELEASE_PIXEL_RIGHT;
        }
        return mecanumDrive.actionBuilder(Constants.B_2_BEGIN_POSE)
                //.strafeTo(new Vector2d(-48,-36))
                .strafeTo(spikeLocation)
                .afterDisp(0.5, pixelDrop.release())
                //.strafeTo(new Vector2d(-36,-60))
                .strafeTo(Constants.B_2_STRAFE_BACK)
                .strafeTo(Constants.B_2_STRAFE_BB)
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(24,-36,0), Math.PI/2)
                .splineToLinearHeading(Constants.B_2_SPLINE_BB, Constants.B_2_HEADING_BB)
                .strafeTo(Constants.B_BEFORE_PARK_BB)
                .afterTime(0.01, () -> clawArm.setTargetPosition(-100))
                .afterDisp(0.5, linearSlides.extendToDropAtSpike())
                //       .turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(24,-60))
                .strafeTo(Constants.B_PARK_BB)

                .build();
    }

    public static Action releasePixelAtBoardAction (MecanumDrive mecanumDrive, String direction, PixelDrop pixelDrop,
                                         LinearSlides linearSlides, ClawArm clawArm, ClawMain clawMain) {
        Context context = new Context();


        return mecanumDrive.actionBuilder(Constants.B_2_BEGIN_POSE)
                //.strafeTo(new Vector2d(-48,-36))
                .strafeTo(new Vector2d(-36.5, 60))
                .afterDisp(0.5, linearSlides.extend())
                .afterTime(0.1, () -> clawArm.setTargetPosition(-400))
                .afterTime(1, clawArm.extendToDropAtBoard())
                .afterTime(3, clawMain.release())

                .build();

    }
}
