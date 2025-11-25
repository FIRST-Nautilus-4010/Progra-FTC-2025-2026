package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    private static final double PPG_POS = -11.6;
    private static final double PGP_POS = 12.2;
    private static final double GPP_POS = 36;

    private static final Pose2d INITIAL_POS_RED_1 = new Pose2d(-23.6 - 9, 24.6 + 9,  Math.PI / 2);
    private static final Pose2d INITIAL_POS_RED_2 = new Pose2d(70 - 8, 24.6 - 9,  Math.PI / 2);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(54.3, -12.9,  -Math.PI / 2))
                        //.strafeTo(new Vector2d(-30, 28)) // solo en pos 1

                        // Disparo de precargadas
                        // Mirar el patron
                        // Activar Intake
                        .waitSeconds(4)

                        .strafeTo(new Vector2d(GPP_POS, 28 + 3)) // GPP
                        .forward(18) // recoger
                        // preparar disparo
                        .strafeTo(new Vector2d(49.7, 17.4)) // GPP
                        .waitSeconds(4)

                        .strafeTo(new Vector2d(PGP_POS, 28 + 3)) // PGP
                        .forward(18) // recoger
                        // preparar disparo
                        .strafeTo(new Vector2d(-7.9, 19.9)) // PGP
                        .waitSeconds(4)

                        .strafeTo(new Vector2d(PPG_POS, 28 + 3)) // PPG
                        .forward(18) // recoger
                        // preparar disparo
                        .strafeTo(new Vector2d(-36.3, 31.5)) // PPG
                        .waitSeconds(4)




                        // Disparar

                        .build());


        Image img = null;
        try { img = ImageIO.read(new File("MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\backgrounds\\field-2025-juice-dark.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}