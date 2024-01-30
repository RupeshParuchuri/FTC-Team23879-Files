package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.firstinspires.ftc.teamcode.robonauts.Constants;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(90), 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(  new Pose2d( new Vector2d(
                        12, 64), -Math.PI/2))
                .strafeTo(new Vector2d(24, 48))
                .strafeTo(new Vector2d(36, 36))
                .turn(Math.toRadians(-90))
                //drop goes here
                .turn(Math.toRadians(90))

               // .afterDisp(0.1, pixelDrop.release())
                //.turn(Math.toRadians(90))

                //.turn(Math.toRadians(-180))
                .strafeTo(new Vector2d(48, 12))
                .turn(Math.toRadians(90))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}