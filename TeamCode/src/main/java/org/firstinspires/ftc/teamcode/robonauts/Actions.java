package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Actions {
    public static Action get_R1_DRIVE_TO_CENTER_PIXEL(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                            ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(-36, -38))
                .build();

    }


    public static Action dropPurplePixel(MecanumDrive mecanumDrive, PixelDrop pixelDrop) {
        return pixelDrop.release();
    }

    public static Action resetPixelArm(MecanumDrive mecanumDrive, PixelDrop pixelDrop) {
        return pixelDrop.resetPixel();
    }

    public static Action dropYellowPixel(MecanumDrive mecanumDrive, ClawMain clawMain) {
        return clawMain.releaseClawAction();
    }


    public static Action get_R1_CENTER_TO_BB(MecanumDrive mecanumDrive, PixelDrop pixelDrop) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo( new Vector2d(-60,-38))
                .strafeTo(new Vector2d(-60,0))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(48,-12))
                .build();

    }

    public static Action RedPark(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                        ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(56,-10))
                .build();
    }

    public static Action BluePark(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                 ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(56,10))
                .build();
    }

    public static Action get_R1_DRIVE_TO_LEFT_PIXEL(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                          ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(-36, -48))
                .strafeTo(new Vector2d(-44, -42))
                .build();
    }

    public static Action get_R1_LEFT_TO_BB(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                             ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(-60,-42))
                .strafeTo(new Vector2d(-60,0))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(36,0))
                .strafeTo(new Vector2d(48,-12))
                .build();

    }




    public static Action get_R1_LEFT_DROP(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                            ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(-36, -48))
                .strafeTo(new Vector2d(-44, -42))

                .afterDisp(4, pixelDrop.release())

                .strafeTo(new Vector2d(-60,-42))
                .afterDisp(2, () -> pixelDrop.reset())

                .strafeTo(new Vector2d(-60,0))


                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(36,0))

                .strafeTo(new Vector2d(48,-12))



                .build();
    }
    public static Action get_R1_DRIVE_TO_RIGHT_PIXEL(MecanumDrive mecanumDrive, PixelDrop pixelDrop, ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(-48, -36))
                .strafeTo(new Vector2d(-38, -36))
                .turn(Math.toRadians(-90))
                .build();
    }

    public static Action get_R4_DRIVE_TO_RIGHT_PIXEL(MecanumDrive mecanumDrive, PixelDrop pixelDrop, ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(-48, -36))
                .strafeTo(new Vector2d(-38, -36))
                .turn(Math.toRadians(-90))
                .build();
    }

    public static Action get_R1_RIGHT_TO_BB(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                          ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(-38, 0))
                .strafeTo(new Vector2d(36,0))
                .strafeTo(new Vector2d(48,-12))
                .build();
    }


    public static Action get_R4_DRIVE_TO_CENTER_PIXEL(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                            ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(12, -38))
                .strafeTo(new Vector2d(10, -42))
                .build();
    }

    public static Action get_R4_CENTER_TO_BB(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                                      ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                 .turn(Math.toRadians(-90))
                 .strafeTo(new Vector2d(36,-36))
                 //.strafeTo(new Vector2d(36,0))
                 .strafeTo(new Vector2d(48,-12))
                .build();
    }

    public static Action get_R4_DRIVE_TO_LEFT_PIXEL(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                                    ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(24, -48))
                .strafeTo(new Vector2d(24, -36))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(12, -36))
                .build();
    }

    public static Action get_R4_LEFT_TO_BB(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                           ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(36, -36))
                .turn(Math.toRadians(90)) //need to turn 180
                //.strafeTo(new Vector2d(36,0))
                .strafeTo(new Vector2d(50,-12))

                .afterDisp(3, clawMain.releaseClawAction())
                .strafeTo(new Vector2d(51,-12))
                .turn(Math.toRadians(90)) //need to turn 180

                .build();
    }

    public static Action get_R4_LEFT_DROP(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                          ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(24, -48))
                .strafeTo(new Vector2d(24, -36))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(12, -36))
                .afterDisp(0.1, pixelDrop.release())
                .strafeTo(new Vector2d(36, -36))
                .turn(Math.toRadians(90)) //need to turn 180
                //.strafeTo(new Vector2d(36,0))
                .strafeTo(new Vector2d(50,-12))

                .afterDisp(3, clawMain.releaseClawAction())
                .strafeTo(new Vector2d(51,-12))
                .turn(Math.toRadians(90)) //need to turn 180

                .build();
    }


    public static Action get_R4_RIGHT_TO_BB(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                           ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)

                .strafeTo(new Vector2d(12, -36))
                .strafeTo(new Vector2d(12, -20))
                .strafeTo(new Vector2d(12, -18))
                .strafeTo(new Vector2d(36, -18))
                .strafeTo(new Vector2d(50,-12))
                .build();
    }
    public static Action get_R4_RIGHT_DROP(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                           ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(12, -36))
                .turn(Math.toRadians(-90))
                //.strafeTo(new Vector2d(9, -36))
                .afterDisp(0.1, pixelDrop.release())
                .strafeTo(new Vector2d(12, -36))
                .strafeTo(new Vector2d(12, -20))
                .strafeTo(new Vector2d(12, -18))
                .strafeTo(new Vector2d(36, -18))
                .strafeTo(new Vector2d(50,-12))
                .afterDisp(3, clawMain.releaseClawAction())
                .strafeTo(new Vector2d(51,-12))

                .build();
    }

    public static Action get_B2_DRIVE_TO_CENTER_PIXEL(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                            ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(-36, 52))
                .strafeTo(new Vector2d(-36, 36))
                .build();
    }


    public static Action get_B2_CENTER_DROP(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                            ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(-36, 52))
                .strafeTo(new Vector2d(-36, 36))
                //.afterDisp(0.1, pixelDrop.release())
                .strafeTo(new Vector2d(-48, 36))
                .strafeTo(new Vector2d(-48, 0))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(36, 0))
                //.strafeTo(new Vector2d(36, 36))
                .strafeTo(new Vector2d(36,0))

                .strafeTo(new Vector2d(48,12))
                .afterDisp(3, clawMain.releaseClawAction())
                .strafeTo(new Vector2d(50,12))

                .build();
    }
    public static Action get_B2_CENTER_DRIVE_TO_BB(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                            ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)

                .strafeTo(new Vector2d(-48, 36))
                .strafeTo(new Vector2d(-48, 0))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(36, 0))
                .strafeTo(new Vector2d(36,0))

                .strafeTo(new Vector2d(48,12))

                .build();
    }



    public static Action get_B2_LEFT_DROP(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                          ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(-46, 48))
                .strafeTo(new Vector2d(-48, 36))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-38, 38))
                .afterDisp(0.1, pixelDrop.release())
                .strafeTo(new Vector2d(-38, 0))
                .strafeTo(new Vector2d(36, 0))
                .strafeTo(new Vector2d(48,12))
                .afterDisp(3, clawMain.releaseClawAction())
                .strafeTo(new Vector2d(50,12))

                .build();
    }

    public static Action get_B2_DRIVE_TO_LEFT_PIXEL(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                          ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)
                .strafeTo(new Vector2d(-46, 48))
                .strafeTo(new Vector2d(-48, 36))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-38, 38))
                /*.afterDisp(0.1, pixelDrop.release())
                .strafeTo(new Vector2d(-38, 0))
                .strafeTo(new Vector2d(36, 0))
                .strafeTo(new Vector2d(48,12))
                .afterDisp(3, clawMain.releaseClawAction())
                .strafeTo(new Vector2d(50,12))*/

                .build();
    }

    public static Action get_B2_LEFT_TO_BB(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                          ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(mecanumDrive.pose)

                .strafeTo(new Vector2d(-38, 0))
                .strafeTo(new Vector2d(36, 0))
                .strafeTo(new Vector2d(48,12))


                .build();
    }

    public static Action get_B2_RIGHT_DROP(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                           ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(-56, 48))
                .strafeTo(new Vector2d(-56, 48))
                .strafeTo(new Vector2d(-56, 34))

                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-60, 34))

                 .afterDisp(0.1, pixelDrop.release())
                .strafeTo(new Vector2d(-58, 0))
                .strafeTo(new Vector2d(36, 0))
                .strafeTo(new Vector2d(48,12))
                .afterDisp(3, clawMain.releaseClawAction())
                .strafeTo(new Vector2d(50,12))

                .build();
    }

    public static Action get_B2_DRIVE_TO_RIGHT_PIXEL(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                           ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(-56, 48))
                .strafeTo(new Vector2d(-56, 48))
                .strafeTo(new Vector2d(-56, 34))

                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-60, 34))



                .build();
    }

    public static Action get_B2_RIGHT_TO_BB(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                           ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)

                .strafeTo(new Vector2d(-58, 0))
                .strafeTo(new Vector2d(36, 0))
                .strafeTo(new Vector2d(48,12))

                .build();
    }
    public static Action get_B3_CENTER_DROP(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                            ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(12, 38))
                .afterDisp(0.1, pixelDrop.release())
                .strafeTo(new Vector2d(12, 36))
                .strafeTo(new Vector2d(36, 36))
                .strafeTo(new Vector2d(50, 12))
                .afterDisp(3, clawMain.releaseClawAction())

                .turn(Math.toRadians(90))

                .build();
    }

    public static Action get_B3_DRIVE_TO_CENTER_PIXEL(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                            ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(12, 38))


                .turn(Math.toRadians(90))

                .build();
    }

    public static Action get_B3_CENTER_DRIVE_TO_BB(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                            ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)

                .strafeTo(new Vector2d(12, 36))
                .strafeTo(new Vector2d(36, 36))
                .strafeTo(new Vector2d(50, 12))
                .build();
    }


    public static Action get_B3_RIGHT_DROP(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                          ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(12, 36))
                .strafeTo(new Vector2d(14, 36))
                .turn(Math.toRadians(-90))
                .afterDisp(0.1, pixelDrop.release())
                //.turn(Math.toRadians(-180))
                .strafeTo(new Vector2d(36, 36))
                .strafeTo(new Vector2d(49, 12))
                .afterDisp(3, clawMain.releaseClawAction())
                .strafeTo(new Vector2d(50, 12))

                .build();

    }

    public static Action get_B3_DRIVE_TO_RIGHT_PIXEL(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                           ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(12, 36))
                .strafeTo(new Vector2d(14, 36))
                .turn(Math.toRadians(-90))


                .build();

    }

    public static Action get_B3_RIGHT_TO_BB(MecanumDrive mecanumDrive, PixelDrop pixelDrop,
                                           ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)

                //.turn(Math.toRadians(-180))
                .strafeTo(new Vector2d(36, 36))
                .strafeTo(new Vector2d(49, 12))


                .build();

    }

    public static Action get_B3_LEFT_DROP(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                           ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(24, 48))
                .strafeTo(new Vector2d(36, 36))
                .turn(Math.toRadians(-90))
                //drop goes here
                .turn(Math.toRadians(90))

                .afterDisp(0.1, pixelDrop.release())
                //.turn(Math.toRadians(90))

                //.turn(Math.toRadians(-180))
                .strafeTo(new Vector2d(50, 12))
                .turn(Math.toRadians(90))
                .afterDisp(3, clawMain.releaseClawAction())
                .strafeTo(new Vector2d(51, 12))
                .build();

    }

    public static Action get_B3_DRIVE_TO_LEFT_PIXEL(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                          ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(24, 48))
                .strafeTo(new Vector2d(36, 36))
                .turn(Math.toRadians(-90))
                //drop goes here
                .turn(Math.toRadians(90))


                .build();

    }

    public static Action get_B3_LEFT_TO_BB(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                          ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)

                .strafeTo(new Vector2d(50, 12))
                .turn(Math.toRadians(90))

                .build();

    }

    public static Action releasePixelFromBlue(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                          ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(  mecanumDrive.pose)
                .strafeTo(new Vector2d(47, 12))
                .afterDisp(2, () -> clawMain.releaseClawAction())
                .strafeTo(new Vector2d(48, 12))

                .build();

    }

    public static Action releasePixelFromRed(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                              ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(  mecanumDrive.pose)
                .strafeTo(new Vector2d(47, -12))
                .afterTime(0.2, clawMain.releaseClawAction())
                .strafeTo(new Vector2d(48, -12))
                .build();

    }

    public static Action ParkRed(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                             ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder(  mecanumDrive.pose)
                .strafeTo(new Vector2d(54, -24))

                .build();

    }

    public static Action ParkBlue(MecanumDrive mecanumDrive,  PixelDrop pixelDrop,
                                 ClawArm clawArm, ClawMain clawMain) {
        return mecanumDrive.actionBuilder( mecanumDrive.pose)
                .strafeTo(new Vector2d(58, 30))

                .build();

    }






    /*public static Action get_R_1_ACTION (MecanumDrive mecanumDrive, String direction, PixelDrop pixelDrop,
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

                .strafeTo(new Vector2d(-36, -48))
                .afterDisp(0.1, () -> pixelDrop.reset())
                .strafeTo(new Vector2d(-36, 0))

                //.strafeTo(new Vector2d(-36, -36))
                //.splineToLinearHeading(new Pose2d( new Vector2d(-36,-24), 0), Constants.R_1_HEADING_BB)
                .turn(Math.toRadians(-90))

                .strafeTo(new Vector2d(24, 0))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(24, -36))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(48, -36))

                .strafeTo(Constants.B_BEFORE_PARK_BB)
                .strafeTo(Constants.R_PARK_BB)

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

        Vector2d spikeLocation = Constants.B_2_STRAFE_RELEASE_PIXEL_RIGHT;
       *//* if (direction.equalsIgnoreCase("left")) {
            spikeLocation = Constants.B_2_STRAFE_RELEASE_PIXEL_LEFT;
        } else if (direction.equalsIgnoreCase("right")) {
            spikeLocation = Constants.B_2_STRAFE_RELEASE_PIXEL_RIGHT;
        }*//*
        return mecanumDrive.actionBuilder(Constants.B_2_BEGIN_POSE)
                //.strafeTo(new Vector2d(-48,-36))
                .strafeTo(spikeLocation)
                .afterDisp(0.5, pixelDrop.release())
                //.strafeTo(new Vector2d(-36,-60))
                .strafeTo(new Vector2d(-36, 36))
                .strafeTo(new Vector2d(-36, 0))

                //.strafeTo(new Vector2d(-36, -36))
                //.splineToLinearHeading(new Pose2d( new Vector2d(-36,-24), 0), Constants.R_1_HEADING_BB)
                .turn(Math.toRadians(90))

                .strafeTo(new Vector2d(24, 0))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(24, 36))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(48, 36))
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

    }*/
}
