package org.firstinspires.ftc.teamcode.subsystems.movement;

import java.util.ArrayDeque;

public class MovementSequenceBuilder {
    private ArrayDeque<Movement> movements;

    public MovementSequenceBuilder() {movements = new ArrayDeque<>();}

    /**
     * Builds the MovementSequence.
     * @return a MovementSequence.
     */
    public MovementSequence build() {return new MovementSequence(movements);}

    /**
     * Appends a forward movement to the MovementSequenceBuilder.
     * @param inches      How far the robot should move in inches.
     */
    public MovementSequenceBuilder forward(double inches) {
        movements.add(new Movement(0, inches, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a backward movement to the MovementSequenceBuilder.
     * @param inches      How far the robot should move in inches.
     */
    public MovementSequenceBuilder backward(double inches) {
        movements.add(new Movement(1, inches, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a left strafe to the MovementSequenceBuilder.
     * @param inches      How far the robot should strafe in inches.
     */
    public MovementSequenceBuilder left(double inches) {
        movements.add(new Movement(2, 0, inches, 0, 0, 0));
        return this;
    }

    /**
     * Appends a right strafe to the MovementSequenceBuilder.
     * @param inches      How far the robot should strafe in inches.
     */
    public MovementSequenceBuilder right(double inches) {
        movements.add(new Movement(3, 0, inches, 0, 0, 0));
        return this;
    }

    /**
     * Appends a left turn to the MovementSequenceBuilder.
     * @param degrees      How much the robot should turn in degrees.
     */
    public MovementSequenceBuilder turnLeft(double degrees) {
        movements.add(new Movement(4, 0, 0, degrees, 0, 0));
        return this;
    }

    /**
     * Appends a left turn to the MovementSequenceBuilder.
     * @param degrees      How much the robot should turn in degrees.
     */
    public MovementSequenceBuilder turnRight(double degrees) {
        movements.add(new Movement(5, 0, 0, degrees, 0, 0));
        return this;
    }

    /**
     * Appends a timeout to the MovementSequenceBuilder.
     * @param ms      The wait time in milliseconds.
     */
    public MovementSequenceBuilder waitFor(long ms) {
        movements.add(new Movement(6, 0, 0, 0, ms, 0));
        return this;
    }

    /**
     * Appends a close claw event to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder closeClaw() {
        movements.add(new Movement(7, 0, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends an open claw event to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder openClaw() {
        movements.add(new Movement(8, 0, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a change arm elevation event to the MovementSequenceBuilder.
     * @param degrees      The angle for the arm to be displaced by in degrees.
     */
    public MovementSequenceBuilder changeElevation(int degrees) {
        movements.add(new Movement(9, 0, 0, 0, 0, degrees));
        return this;
    }
}
