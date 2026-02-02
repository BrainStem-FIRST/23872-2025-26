package org.firstinspires.ftc.teamcode.utils;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class BallTrackerNew {




    private static final int TICKS_PER_REV = 8192;
    private static final int TICKS_PER_SLOT = 2730;
    private Spindexer spindexer;
    private Telemetry telemetry;
    private int spindTick; // spind ticks from encoder
    private int spindAbsTicks;
    public enum BallColor {
        PURPLE,
        GREEN,
        EMPTY
    }


    public final List<BallColor> motif1 = new ArrayList<>(Arrays.asList(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN));
    public final List<BallColor> motif2 = new ArrayList<>(Arrays.asList(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE));
    public final List<BallColor> motif3 = new ArrayList<>(Arrays.asList(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE));



    public List<BallColor> targetMotif;
    public BallTrackerNew(Spindexer spindexer) {
        this.spindexer = spindexer;
        this.targetMotif = motif1;
    }
    public class Slot {
        public String name;
        public BallColor color;
        public int pos; // 0 - whatever ------- actual enc positon
        public int currentAbsPos; // 0 -8192
        public Slot(String name, int pos) {
            this.name = name;
            this.pos = pos;
            this.color = BallColor.EMPTY;
            this.currentAbsPos = 0;
        }
    }
    public Slot slotA = new Slot("Slot A", 0);
    public Slot slotC = new Slot("Slot C", 2730);
    public Slot slotB = new Slot("Slot B", 5460);

    public void update() {
        spindTick = spindexer.spindexerMotor.getCurrentPosition();
        spindAbsTicks = calcAbsPos(spindTick);
        updateSlotTicks(slotA);
        updateSlotTicks(slotB);
        updateSlotTicks(slotC);
    }
    private void updateSlotTicks(Slot slot) {
        int calculatedPos = spindTick + slot.pos;
        slot.currentAbsPos = calcAbsPos(calculatedPos);
    }
    private int calcAbsPos(int ticks) {
        return ticks % TICKS_PER_REV;
    }
    public void setSlotClr(Slot slot, BallColor color) {
        slot.color = color;
    }
    public Slot getSlotAtShootingPos() {
        if (slotA.currentAbsPos < 2830 && slotA.currentAbsPos > 2630) return slotA;
        if (slotB.currentAbsPos < 2830 && slotB.currentAbsPos > 2630) return slotB;
        return slotC;
    }

    public Slot getSlotAtCollectPos() {
        if (slotA.currentAbsPos < 100 || slotA.currentAbsPos > 8092) return slotA;
        if (slotB.currentAbsPos < 100 || slotB.currentAbsPos > 8092) return slotB;
        return slotC;
    }

    public Slot getSlotAtEndPost() {
        if (slotA.currentAbsPos < 100 || slotA.currentAbsPos > 8092) return slotA;
        if (slotB.currentAbsPos < 100 || slotB.currentAbsPos > 8092) return slotB;
        return slotC;

        // CHANGE
    }



    public int getBestRotation() {

        List<BallColor> seqA = Arrays.asList(slotC.color, slotA.color, slotB.color); // dont move

        List<BallColor> seqB = Arrays.asList(slotA.color, slotB.color,slotC.color); // 60 deg

        List<BallColor> seqC = Arrays.asList(slotB.color, slotC.color, slotA.color); // 120 deg
        // get scores
        int scoreA = calculateScore(seqA);
        int scoreB = calculateScore(seqB);
        int scoreC = calculateScore(seqC);
        // find best one
        if (scoreA >= scoreB && scoreA >= scoreC) {
            return (2730 - slotC.currentAbsPos + 8192) % 8192;
        } else if (scoreB >= scoreC) {
            return (2730 - slotA.currentAbsPos + 8192) % 8192;
        } else {
            return (2730 - slotB.currentAbsPos + 8192) % 8192;
        }
    }

    private int calculateScore(List<BallColor> slotSeq) {
        int score = 0;
        for (int i = 0; i < 3; i++) {
            BallColor ball = slotSeq.get(i);
            BallColor goal = targetMotif.get(i);


            if (ball != BallColor.EMPTY && ball == goal) {
                score++;
            }
        }
        return score;
    }

    public void addBall(BallColor color) {
        getSlotAtCollectPos().color = color;

    }
}