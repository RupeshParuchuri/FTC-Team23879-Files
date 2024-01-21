package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Constants {

    public static final Pose2d R_1_BEGIN_POSE = new Pose2d( new Vector2d(-36, -60), Math.PI/2);
    public static final Vector2d R_1_STRAFE_RELEASE_PIXEL_LEFT = new Vector2d(-44, -36);
    public static final Vector2d R_1_STRAFE_RELEASE_PIXEL_RIGHT = new Vector2d(-20, -36);
    public static final Vector2d R_1_STRAFE_RELEASE_PIXEL_CENTER = new Vector2d(-36, -20);
    public static final Vector2d R_1_STRAFE_BACK = new Vector2d(-36, -60);
    public static final Vector2d R_1_STRAFE_BB = new Vector2d(36, -55);
    public static final Pose2d R_1_SPLINE_BB = new Pose2d( new Vector2d(60, -36), 0);
    public static final double R_1_HEADING_BB = Math.PI/2;


    public static final Pose2d R_4_BEGIN_POSE = new Pose2d( new Vector2d(36, -60), Math.PI/2);
    public static final Vector2d R_4_STRAFE_RELEASE_PIXEL_LEFT = new Vector2d(28, -36);
    public static final Vector2d R_4_STRAFE_RELEASE_PIXEL_RIGHT = new Vector2d(52, -36);
    public static final Vector2d R_4_STRAFE_RELEASE_PIXEL_CENTER = new Vector2d(30, 24);
    public static final Vector2d R_4_STRAFE_BACK = new Vector2d(36, -60);
    public static final Pose2d R_4_SPLINE_BB = new Pose2d( new Vector2d(72, -36), 0);
    public static final double R_4_HEADING_BB = Math.PI/2;

    public static final Pose2d B_3_BEGIN_POSE = new Pose2d( new Vector2d(36, 60), -Math.PI/2);
    public static final Vector2d B_3_STRAFE_RELEASE_PIXEL_LEFT = new Vector2d(44, 36);
    public static final Vector2d B_3_STRAFE_RELEASE_PIXEL_RIGHT = new Vector2d(20, 36);
    public static final Vector2d B_3_STRAFE_RELEASE_PIXEL_CENTER = new Vector2d(30, 24);
    public static final Vector2d B_3_STRAFE_BACK = new Vector2d(36, 60);
    public static final Pose2d B_3_SPLINE_BB = new Pose2d( new Vector2d(72, 36), 0);
    public static final double B_3_HEADING_BB = -Math.PI/2;

    public static final Pose2d B_2_BEGIN_POSE = new Pose2d( new Vector2d(36, -60), -Math.PI/2);
    public static final Vector2d B_2_STRAFE_RELEASE_PIXEL_LEFT = new Vector2d(-20, 36);
    public static final Vector2d B_2_STRAFE_RELEASE_PIXEL_RIGHT = new Vector2d(-44, 36);
    public static final Vector2d B_2_STRAFE_RELEASE_PIXEL_CENTER = new Vector2d(-36, 24);
    public static final Vector2d B_2_STRAFE_BACK = new Vector2d(-36, 60);
    public static final Vector2d B_2_STRAFE_BB = new Vector2d(48, 60);
    public static final Pose2d B_2_SPLINE_BB = new Pose2d( new Vector2d(72, 36), 0);
    public static final double B_2_HEADING_BB = -Math.PI/2;

    public static final Vector2d R_PARK_BB = new Vector2d(60, -12);
    public static final Vector2d B_PARK_BB = new Vector2d(60, 12);
}
