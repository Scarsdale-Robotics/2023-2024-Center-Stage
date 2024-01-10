package org.firstinspires.ftc.teamcode.subsystems.movement;

import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;

import java.util.ArrayDeque;

public class MovementSequenceBuilder {
    private final ArrayDeque<Movement> movements;

    public MovementSequenceBuilder() {movements = new ArrayDeque<>();}

    /**
     * Builds the MovementSequence.
     * @return a MovementSequence.
     */
    public MovementSequence build() {return new MovementSequence(movements);}

    public MovementSequenceBuilder alignWithWhitePixel() {
        movements.add(new Movement(Movement.MovementType.WHITE_PXL_ALIGN, 0, 0, 0, 0, 0));
        return this;
    }

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
     * Appends a forward-left movement to the MovementSequenceBuilder.
     * @param inchesForward      How much the robot should move forward in inches.
     * @param inchesLeft         How much the robot should strafe left in inches.
     */
    public MovementSequenceBuilder forwardLeft(double inchesForward, double inchesLeft) {
        movements.add(new Movement(Movement.MovementType.FORWARD_LEFT, inchesForward, inchesLeft, 0, 0, 0));
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
     * Appends a backward-left movement to the MovementSequenceBuilder.
     * @param inchesBackward     How much the robot should move backward in inches.
     * @param inchesLeft         How much the robot should strafe left in inches.
     */
    public MovementSequenceBuilder backwardLeft(double inchesBackward, double inchesLeft) {
        movements.add(new Movement(Movement.MovementType.BACKWARD_LEFT, inchesBackward, inchesLeft, 0, 0, 0));
        return this;
    }

    /**
     * Appends a backward-right movement to the MovementSequenceBuilder.
     * @param inchesBackward     How much the robot should move backward in inches.
     * @param inchesRight        How much the robot should strafe right in inches.
     */
    public MovementSequenceBuilder backwardRight(double inchesBackward, double inchesRight) {
        movements.add(new Movement(Movement.MovementType.BACKWARD_RIGHT, inchesBackward, inchesRight, 0, 0, 0));
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
     * Appends a rest elbow event to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder restElbow() {
        movements.add(new Movement(Movement.MovementType.REST_ELBOW, 0, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a flip elbow event to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder flipElbow() {
        movements.add(new Movement(Movement.MovementType.FLIP_ELBOW, 0, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends an open claw event for both claws to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder openBothClaws() {
        return this.openLeftClaw().openRightClaw();
    }

    /**
     * Appends a close claw event for both claws to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder closeBothClaws() {
        return this.closeLeftClaw().closeRightClaw();
    }

    /**
     * Appends an open left claw event for both claws to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder openLeftClaw() {
        movements.add(new Movement(Movement.MovementType.OPEN_LEFT_CLAW, 0, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends an open right claw event for both claws to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder openRightClaw() {
        movements.add(new Movement(Movement.MovementType.OPEN_RIGHT_CLAW, 0, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a close left claw event for both claws to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder closeLeftClaw() {
        movements.add(new Movement(Movement.MovementType.CLOSE_LEFT_CLAW, 0, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a close right claw event for both claws to the MovementSequenceBuilder.
     */
    public MovementSequenceBuilder closeRightClaw() {
        movements.add(new Movement(Movement.MovementType.CLOSE_RIGHT_CLAW, 0, 0, 0, 0, 0));
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

    ////////////////////////////////////
    // METHOD OVERLOADING FOR LINKING //
    ////////////////////////////////////

    /**
     * Appends a forward movement to the MovementSequenceBuilder.
     * @param inches      How far the robot should move in inches.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder forward(double inches, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.FORWARD, inches, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a backward movement to the MovementSequenceBuilder.
     * @param inches      How far the robot should move in inches.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder backward(double inches, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.BACKWARD, inches, 0, 0, 0, 0));
        return this;
    }

    /**
     * Appends a left strafe to the MovementSequenceBuilder.
     * @param inches      How far the robot should strafe in inches.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder left(double inches, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.STRAFE_LEFT, 0, inches, 0, 0, 0));
        return this;
    }

    /**
     * Appends a right strafe to the MovementSequenceBuilder.
     * @param inches      How far the robot should strafe in inches.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder right(double inches, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.STRAFE_RIGHT, 0, inches, 0, 0, 0));
        return this;
    }

    /**
     * Appends a left turn to the MovementSequenceBuilder.
     * @param degrees      How much the robot should turn in degrees.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder turnLeft(double degrees, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.TURN_LEFT, 0, 0, degrees, 0, 0));
        return this;
    }

    /**
     * Appends a left turn to the MovementSequenceBuilder.
     * @param degrees      How much the robot should turn in degrees.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder turnRight(double degrees, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.TURN_RIGHT, 0, 0, degrees, 0, 0));
        return this;
    }

    /**
     * Appends a forward-left movement to the MovementSequenceBuilder.
     * @param inchesForward      How much the robot should move forward in inches.
     * @param inchesLeft         How much the robot should strafe left in inches.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder forwardLeft(double inchesForward, double inchesLeft, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.FORWARD_LEFT, inchesForward, inchesLeft, 0, 0, 0));
        return this;
    }

    /**
     * Appends a forward-right movement to the MovementSequenceBuilder.
     * @param inchesForward      How much the robot should move forward in inches.
     * @param inchesRight        How much the robot should strafe right in inches.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder forwardRight(double inchesForward, double inchesRight, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.FORWARD_RIGHT, inchesForward, inchesRight, 0, 0, 0));
        return this;
    }

    /**
     * Appends a backward-left movement to the MovementSequenceBuilder.
     * @param inchesBackward     How much the robot should move backward in inches.
     * @param inchesLeft         How much the robot should strafe left in inches.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder backwardLeft(double inchesBackward, double inchesLeft, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.BACKWARD_LEFT, inchesBackward, inchesLeft, 0, 0, 0));
        return this;
    }

    /**
     * Appends a backward-right movement to the MovementSequenceBuilder.
     * @param inchesBackward     How much the robot should move backward in inches.
     * @param inchesRight        How much the robot should strafe right in inches.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder backwardRight(double inchesBackward, double inchesRight, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.BACKWARD_RIGHT, inchesBackward, inchesRight, 0, 0, 0));
        return this;
    }

    /**
     * Appends a lower arm event to the MovementSequenceBuilder.
     * @param degrees      The angle for the arm to be elevated in degrees.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder lowerArm(double degrees, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.LOWER_ARM, 0, 0, 0, degrees, 0));
        return this;
    }

    /**
     * Appends a raise arm event to the MovementSequenceBuilder.
     * @param degrees      The angle for the arm to be elevated in degrees.
     * @param withPrevious      If this movement should be executed at the same time as the previous movement is executed.
     */
    public MovementSequenceBuilder raiseArm(double degrees, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            movements.getLast().linkWithNext();
        movements.add(new Movement(Movement.MovementType.RAISE_ARM, 0, 0, 0, degrees, 0));
        return this;
    }

}
