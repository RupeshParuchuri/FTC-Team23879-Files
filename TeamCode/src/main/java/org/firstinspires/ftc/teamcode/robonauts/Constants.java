package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Constants {

    public static final Pose2d R_1_BEGIN_POSE = new Pose2d( new Vector2d(-36, -60), Math.PI/2);
    public static final Vector2d R_1_STRAFE_RELEASE_PIXEL_LEFT = new Vector2d(-48  , -48);
    public static final Vector2d R_1_STRAFE_RELEASE_PIXEL_RIGHT = new Vector2d(-18, -34);
    public static final Vector2d R_1_STRAFE_RELEASE_PIXEL_CENTER = new Vector2d(-36, -36);
    public static final Vector2d R_1_STRAFE_BACK = new Vector2d(-36, -60);
    public static final Vector2d R_1_STRAFE_BB = new Vector2d(36, -58);
    public static final Pose2d R_1_SPLINE_BB = new Pose2d( new Vector2d(45, -36), 0);
    public static final double R_1_HEADING_BB = Math.PI/2;

    public static final Vector2d B_BEFORE_PARK_BB = new Vector2d(45, 8);

    public static final Pose2d R_4_BEGIN_POSE = new Pose2d( new Vector2d(12, -60), Math.PI/2);
    public static final Vector2d R_4_STRAFE_RELEASE_PIXEL_LEFT = new Vector2d(10, -36);
    public static final Vector2d R_4_STRAFE_RELEASE_PIXEL_RIGHT = new Vector2d(10, -36);
    public static final Vector2d R_4_STRAFE_RELEASE_PIXEL_CENTER = new Vector2d(10, -36);
    public static final Vector2d R_4_STRAFE_BACK = new Vector2d(36, -50);
    public static final Pose2d R_4_SPLINE_BB = new Pose2d( new Vector2d(45, -36), 0);
    public static final double R_4_HEADING_BB = Math.PI/2;

    public static final Pose2d B_3_BEGIN_POSE = new Pose2d( new Vector2d(12, 60), -Math.PI/2);
    public static final Vector2d B_3_STRAFE_RELEASE_PIXEL_LEFT = new Vector2d(44, 36);
    public static final Vector2d B_3_STRAFE_RELEASE_PIXEL_RIGHT = new Vector2d(20, 36);
    public static final Vector2d B_3_STRAFE_RELEASE_PIXEL_CENTER = new Vector2d(12,30);
    public static final Vector2d B_3_STRAFE_BACK = new Vector2d(36, 60);
    public static final Pose2d B_3_SPLINE_BB = new Pose2d( new Vector2d(45, 36), 0);
    public static final double B_3_HEADING_BB = -Math.PI/2;

    public static final Pose2d B_2_BEGIN_POSE = new Pose2d( new Vector2d(-36, 60), -Math.PI/2);
    public static final Vector2d B_2_STRAFE_RELEASE_PIXEL_LEFT = new Vector2d(-20, 32);
    public static final Vector2d B_2_STRAFE_RELEASE_PIXEL_RIGHT = new Vector2d(-44, 32);
    public static final Vector2d B_2_STRAFE_RELEASE_PIXEL_CENTER = new Vector2d(-36, 32);
    public static final Vector2d B_2_STRAFE_BACK = new Vector2d(-36, 55);
    public static final Vector2d B_2_STRAFE_BB = new Vector2d(24, 55);
    public static final Pose2d B_2_SPLINE_BB = new Pose2d( new Vector2d(45, 32), 0);
    public static final double B_2_HEADING_BB = -Math.PI/2;

    public static final Vector2d R_PARK_BB = new Vector2d(60, 0);
    public static final Vector2d B_PARK_BB = new Vector2d(60, 8);


}
