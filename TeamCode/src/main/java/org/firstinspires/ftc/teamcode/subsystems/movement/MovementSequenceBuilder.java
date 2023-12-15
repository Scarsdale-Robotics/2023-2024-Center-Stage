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
        movements.add(new Movement(Movement.MovementType.FORWARD, inches, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a backward movement to the MovementSequenceBuilder.
     * @param inches      How far the robot should move in inches.
     */
    public MovementSequenceBuilder backward(double inches) {
        movements.add(new Movement(Movement.MovementType.BACKWARD, inches, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a left strafe to the MovementSequenceBuilder.
     * @param inches      How far the robot should strafe in inches.
     */
    public MovementSequenceBuilder left(double inches) {
        movements.add(new Movement(Movement.MovementType.STRAFE_LEFT, 0, inches, 0, 0, 0));
        return this;
    }

    /**
     * Appends a right strafe to the MovementSequenceBuilder.
     * @param inches      How far the robot should strafe in inches.
     */
    public MovementSequenceBuilder right(double inches) {
        movements.add(new Movement(Movement.MovementType.STRAFE_RIGHT, 0, inches, 0, 0, 0));
        return this;
    }

    /**
     * Appends a left turn to the MovementSequenceBuilder.
     * @param degrees      How much the robot should turn in degrees.
     */
    public MovementSequenceBuilder turnLeft(double degrees) {
        movements.add(new Movement(Movement.MovementType.TURN_LEFT, 0, 0, degrees, 0, 0));
        return this;
    }

    /**
     * Appends a left turn to the MovementSequenceBuilder.
     * @param degrees      How much the robot should turn in degrees.
     */
    public MovementSequenceBuilder turnRight(double degrees) {
        movements.add(new Movement(Movement.MovementType.TURN_RIGHT, 0, 0, degrees, 0, 0));
        return this;
    }

    /**
     * Appends a forward-right movement to the MovementSequenceBuilder.
     * @param inchesForward      How much the robot should move forward in inches.
     * @param inchesRight        How much the robot should strafe right in inches.
     */
    public MovementSequenceBuilder forwardRight(double inchesForward, double inchesRight) {
        movements.add(new Movement(Movement.MovementType.FORWARD_RIGHT, inchesForward, inchesRight, 0, 0, 0));
        return this;
    }

    /**
     * Appends a forward-left movement to the MovementSequenceBuilder.
     * @param inchesForward      How much the robot should move forward in inches.
     * @param inchesLeft         How much the robot should strafe left in inches.
     */
    public MovementSequenceBuilder forwardLeft(double inchesForward, double inchesLeft) {
        movements.add(new Movement(Movement.MovementType.FORWARD_LEFT, inchesForward, inchesLeft, 0, 0, 0));
        return this;
    }

    /**
     * Appends a backward-right movement to the MovementSequenceBuilder.
     * @param inchesBackward     How much the robot should move backward in inches.
     * @param inchesRight        How much the robot should strafe right in inches.
     */
    public MovementSequenceBuilder backwardRight(double inchesBackward, double inchesRight) {
        movements.add(new Movement(Movement.MovementType.BACKWARD_RIGHT, inchesForward, inchesRight, 0, 0, 0));
        return this;
    }

    /**
     * Appends a backward-left movement to the MovementSequenceBuilder.
     * @param inchesBackward     How much the robot should move backward in inches.
     * @param inchesLeft         How much the robot should strafe left in inches.
     */
    public MovementSequenceBuilder backwardLeft(double inchesBackward, double inchesLeft) {
        movements.add(new Movement(Movement.MovementType.BACKWARD_LEFT, inchesBackward, inchesLeft, 0, 0, 0));
        return this;
    }

    /**
     * Appends a timeout to the MovementSequenceBuilder.
     * @param ms      The wait time in milliseconds.
     */
    public MovementSequenceBuilder sleepFor(long ms) {
        movements.add(new Movement(Movement.MovementType.DELAY, 0, 0, 0, 0, ms));
        return this;
    }

    /**
     * Appends a close claw event to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder closeClaw() {
        movements.add(new Movement(Movement.MovementType.CLOSE_CLAW, 0, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends an open claw event to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder openClaw() {
        movements.add(new Movement(Movement.MovementType.OPEN_CLAW, 0, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a lower arm event to the MovementSequenceBuilder.
     * @param degrees      The angle for the arm to be elevated in degrees.
     */
    public MovementSequenceBuilder lowerArm(double degrees) {
        movements.add(new Movement(Movement.MovementType.LOWER_ARM, 0, 0, 0, degrees, 0));
        return this;
    }

    /**
     * Appends a raise arm event to the MovementSequenceBuilder.
     * @param degrees      The angle for the arm to be elevated in degrees.
     */
    public MovementSequenceBuilder raiseArm(double degrees) {
        movements.add(new Movement(Movement.MovementType.RAISE_ARM, 0, 0, 0, degrees, 0));
        return this;
    }
}
