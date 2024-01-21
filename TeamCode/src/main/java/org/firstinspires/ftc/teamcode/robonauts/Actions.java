package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Actions {
    public static Action get_R_1_ACTION (MecanumDrive mecanumDrive, String direction) {
        Vector2d spikeLocation = Constants.R_1_STRAFE_RELEASE_PIXEL_CENTER;
        if (direction.equalsIgnoreCase("left")) {
            spikeLocation = Constants.R_1_STRAFE_RELEASE_PIXEL_LEFT;
        } else if (direction.equalsIgnoreCase("right")) {
            spikeLocation = Constants.R_1_STRAFE_RELEASE_PIXEL_RIGHT;
        }

        return mecanumDrive.actionBuilder(Constants.R_1_BEGIN_POSE)
                //.strafeTo(new Vector2d(-48,-36))
                .strafeTo(spikeLocation)
                //.afterDisp(10, pixelDrop.release())
                //.strafeTo(new Vector2d(-36,-60))
                .strafeTo(Constants.R_1_STRAFE_BACK)
                .strafeTo(Constants.R_1_STRAFE_BB)
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(24,-36,0), Math.PI/2)
                .splineToLinearHeading(Constants.R_1_SPLINE_BB, Constants.R_1_HEADING_BB)
                .strafeTo(Constants.R_PARK_BB)
                //       .turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(24,-60))
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

    public static Action get_B_2_ACTION (MecanumDrive mecanumDrive, String direction, PixelDrop pixelDrop) {
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
                .afterDisp(10, pixelDrop.release())
                //.strafeTo(new Vector2d(-36,-60))
                .strafeTo(Constants.B_2_STRAFE_BACK)
                .strafeTo(Constants.B_2_STRAFE_BB)
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(24,-36,0), Math.PI/2)
                .splineToLinearHeading(Constants.B_2_SPLINE_BB, Constants.B_2_HEADING_BB)
                .strafeTo(Constants.B_BEFORE_PARK_BB)
                //       .turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(24,-60))
                .strafeTo(Constants.B_PARK_BB)

                .build();
    }
}
