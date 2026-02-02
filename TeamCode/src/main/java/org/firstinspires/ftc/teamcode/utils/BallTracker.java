package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
@Deprecated
public class BallTracker {
    // HAS TO BE IN SHOOT POS BEFORE
    //only tracks state - to be tested
    //declarations
    private Spindexer spindexer;
    private Telemetry telemetry;

    public List<String> targetMotif; // add color sensor stuff to here

   public List<String> pos = new ArrayList<>(Arrays.asList("GREEN", "NA", "PURPLE", "NA", "PURPLE", "NA"));

   public final List<String> motif1 = new ArrayList<>(Arrays.asList("PURPLE", "PURPLE", "GREEN")); // purple, purple, green
     public final List<String> motif2 = new ArrayList<>(Arrays.asList("PURPLE", "GREEN", "PURPLE")); // purple green purple
    public final List<String> motif3 = new ArrayList<>(Arrays.asList("GREEN", "PURPLE", "PURPLE")); // green, purple, purple

    public int getBestRotation() {

        if ((Objects.equals(pos.get(1), "NA")) || (Objects.equals(pos.get(3), "NA")) || (Objects.equals(pos.get(5), "NA"))) {
            return 0;
        }
        // shoot immediately
        List<String> seqA = Arrays.asList(pos.get(3), pos.get(1), pos.get(5));
        // rotate 120
        List<String> seqB = Arrays.asList(pos.get(1), pos.get(5), pos.get(3));
        // rotate 240
        List<String> seqC = Arrays.asList(pos.get(5), pos.get(3), pos.get(1));
        // get scores
        int scoreA = calculateScore(seqA, targetMotif);
        int scoreB = calculateScore(seqB, targetMotif);
        int scoreC = calculateScore(seqC, targetMotif);
        // find best one
        if (scoreA >= scoreB && scoreA >= scoreC) {
            return 0;
        } else if (scoreB >= scoreC) {
            return 96;
        } else {
            return 192;
        }
    }

    private int calculateScore(List<String> option, List<String> target) {
        int score = 0;
        for (int i = 0; i < 3; i++) {
            // check
            String ball = option.get(i);
            String goal = target.get(i);
            if (!ball.equals("NA") && !ball.equals("EMPTY") && ball.equals(goal)) {
                score++;
            }
        }
        return score;
    }

    public void addBall(String color) {

        pos.set(0, color);

    }

    public void rotated60() {Collections.rotate(pos, 1);}
    public void rotatedNeg60() {Collections.rotate(pos, -1);}
    public void rotated120() { Collections.rotate(pos,2);}

}
