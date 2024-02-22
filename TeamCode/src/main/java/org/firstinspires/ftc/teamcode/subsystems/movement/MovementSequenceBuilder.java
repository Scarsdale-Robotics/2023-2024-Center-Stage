package org.firstinspires.ftc.teamcode.subsystems.movement;

import java.util.ArrayDeque;

public class MovementSequenceBuilder {
    private final ArrayDeque<Movement> movements;

    public MovementSequenceBuilder() {
        movements = new ArrayDeque<>();
    }

    /**
     * Builds the MovementSequence.
     *
     * @return a MovementSequence.
     */
    public MovementSequence build() {
        return new MovementSequence(movements);
    }

    public ArrayDeque<Movement> getMovements() {
        return movements;
    }

    /**
     * Sets true the linkedToNext boolean of the last Movement in this
     * MovementSequenceBuilder.
     */
    public void linkLastMovement() {
        movements.getLast().linkWithNext();
    }

    /**
     * Sets true the ignoreEndVelocity boolean of the last Movement in this
     * MovementSequenceBuilder.
     */
    public void setLastIgnoredEndVelocity() {
        movements.getLast().setIgnoredEndVelocity();
    }

    /**
     * Sets true the ignoreStartVelocity boolean of the last Movement in this
     * MovementSequenceBuilder.
     */
    public void setLastIgnoredStartVelocity() {
        movements.getLast().setIgnoredStartVelocity();
    }

    /**
     * Appends a movement to this MovementSequenceBuilder.
     *
     * @param movement The Movement to be appended.
     */
    public MovementSequenceBuilder append(Movement movement) {
        movements.add(movement);
        return this;
    }

    /**
     * Appends the movements of a MovementSequence to the end of this
     * MovementSequenceBuilder.
     *
     * @param movementSequence The MovementSequence to be appended.
     */
    public MovementSequenceBuilder append(MovementSequence movementSequence) {
        movements.addAll(movementSequence.movements);
        return this;
    }

    /**
     * Appends the movements of a MovementSequenceBuilder to the end of this
     * MovementSequenceBuilder.
     *
     * @param movementSequenceBuilder The MovementSequenceBuilder to be appended.
     */
    public MovementSequenceBuilder append(MovementSequenceBuilder movementSequenceBuilder) {
        movements.addAll(movementSequenceBuilder.movements);
        return this;
    }

    /**
     * Appends the movements interpreted from a string to the end of this
     * MovementSequenceBuilder.
     *
     * @param s The string whose movements are to be interpreted and appended.
     */
    public MovementSequenceBuilder append(String s) {
        return this.append(MovementStringInterpreter.toMovementSequenceBuilder(s));
    }

    /**
     * Appends a forward movement to this MovementSequenceBuilder.
     *
     * @param inches How far the robot should move in inches.
     */
    public MovementSequenceBuilder forward(double inches) {
        // ignore this start velocity if previous end was ignored
        boolean ignoreStartVelocity = !movements.isEmpty() && movements.getLast().ignoreEndVelocity;
        // ignore this end velocity if linked to an ignored end velocity
        boolean ignoreEndVelocity = ignoreStartVelocity && movements.getLast().linkedToNext;

        movements.add(new Movement(Movement.MovementType.FORWARD, inches, 0, 0, 0, 0, 0, new AprilTagValues()));

        if (ignoreStartVelocity)
            setLastIgnoredStartVelocity();
        if (ignoreEndVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a backward movement to this MovementSequenceBuilder.
     *
     * @param inches How far the robot should move in inches.
     */
    public MovementSequenceBuilder backward(double inches) {
        // ignore this start velocity if previous end was ignored
        boolean ignoreStartVelocity = !movements.isEmpty() && movements.getLast().ignoreEndVelocity;
        // ignore this end velocity if linked to an ignored end velocity
        boolean ignoreEndVelocity = ignoreStartVelocity && movements.getLast().linkedToNext;

        movements.add(new Movement(Movement.MovementType.BACKWARD, inches, 0, 0, 0, 0, 0, new AprilTagValues()));

        if (ignoreStartVelocity)
            setLastIgnoredStartVelocity();
        if (ignoreEndVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a left strafe to this MovementSequenceBuilder.
     *
     * @param inches How far the robot should strafe in inches.
     */
    public MovementSequenceBuilder left(double inches) {
        // ignore this start velocity if previous end was ignored
        boolean ignoreStartVelocity = !movements.isEmpty() && movements.getLast().ignoreEndVelocity;
        // ignore this end velocity if linked to an ignored end velocity
        boolean ignoreEndVelocity = ignoreStartVelocity && movements.getLast().linkedToNext;

        movements.add(new Movement(Movement.MovementType.STRAFE_LEFT, 0, inches, 0, 0, 0, 0, new AprilTagValues()));

        if (ignoreStartVelocity)
            setLastIgnoredStartVelocity();
        if (ignoreEndVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a right strafe to this MovementSequenceBuilder.
     *
     * @param inches How far the robot should strafe in inches.
     */
    public MovementSequenceBuilder right(double inches) {
        // ignore this start velocity if previous end was ignored
        boolean ignoreStartVelocity = !movements.isEmpty() && movements.getLast().ignoreEndVelocity;
        // ignore this end velocity if linked to an ignored end velocity
        boolean ignoreEndVelocity = ignoreStartVelocity && movements.getLast().linkedToNext;

        movements.add(new Movement(Movement.MovementType.STRAFE_RIGHT, 0, inches, 0, 0, 0, 0, new AprilTagValues()));

        if (ignoreStartVelocity)
            setLastIgnoredStartVelocity();
        if (ignoreEndVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a left turn to this MovementSequenceBuilder.
     *
     * @param degrees How much the robot should turn in degrees.
     */
    public MovementSequenceBuilder turnLeft(double degrees) {
        movements.add(new Movement(Movement.MovementType.TURN_LEFT, 0, 0, degrees, 0, 0, 0, new AprilTagValues()));
        return this;
    }

    /**
     * Appends a left turn to this MovementSequenceBuilder.
     *
     * @param degrees How much the robot should turn in degrees.
     */
    public MovementSequenceBuilder turnRight(double degrees) {
        movements.add(new Movement(Movement.MovementType.TURN_RIGHT, 0, 0, degrees, 0, 0, 0, new AprilTagValues()));
        return this;
    }

    /**
     * Appends a forward-left movement to this MovementSequenceBuilder.
     *
     * @param inchesForward How much the robot should move forward in inches.
     * @param inchesLeft    How much the robot should strafe left in inches.
     */
    public MovementSequenceBuilder forwardLeft(double inchesForward, double inchesLeft) {
        // ignore this start velocity if previous end was ignored
        boolean ignoreStartVelocity = !movements.isEmpty() && movements.getLast().ignoreEndVelocity;
        // ignore this end velocity if linked to an ignored end velocity
        boolean ignoreEndVelocity = ignoreStartVelocity && movements.getLast().linkedToNext;

        movements.add(new Movement(Movement.MovementType.FORWARD_LEFT, inchesForward, inchesLeft, 0, 0, 0, 0, new AprilTagValues()));

        if (ignoreStartVelocity)
            setLastIgnoredStartVelocity();
        if (ignoreEndVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a forward-right movement to this MovementSequenceBuilder.
     *
     * @param inchesForward How much the robot should move forward in inches.
     * @param inchesRight   How much the robot should strafe right in inches.
     */
    public MovementSequenceBuilder forwardRight(double inchesForward, double inchesRight) {
        // ignore this start velocity if previous end was ignored
        boolean ignoreStartVelocity = !movements.isEmpty() && movements.getLast().ignoreEndVelocity;
        // ignore this end velocity if linked to an ignored end velocity
        boolean ignoreEndVelocity = ignoreStartVelocity && movements.getLast().linkedToNext;

        movements.add(new Movement(Movement.MovementType.FORWARD_RIGHT, inchesForward, inchesRight, 0, 0, 0, 0, new AprilTagValues()));

        if (ignoreStartVelocity)
            setLastIgnoredStartVelocity();
        if (ignoreEndVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a backward-left movement to this MovementSequenceBuilder.
     *
     * @param inchesBackward How much the robot should move backward in inches.
     * @param inchesLeft     How much the robot should strafe left in inches.
     */
    public MovementSequenceBuilder backwardLeft(double inchesBackward, double inchesLeft) {
        // ignore this start velocity if previous end was ignored
        boolean ignoreStartVelocity = !movements.isEmpty() && movements.getLast().ignoreEndVelocity;
        // ignore this end velocity if linked to an ignored end velocity
        boolean ignoreEndVelocity = ignoreStartVelocity && movements.getLast().linkedToNext;

        movements.add(new Movement(Movement.MovementType.BACKWARD_LEFT, inchesBackward, inchesLeft, 0, 0, 0, 0, new AprilTagValues()));

        if (ignoreStartVelocity)
            setLastIgnoredStartVelocity();
        if (ignoreEndVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a backward-right movement to this MovementSequenceBuilder.
     *
     * @param inchesBackward How much the robot should move backward in inches.
     * @param inchesRight    How much the robot should strafe right in inches.
     */
    public MovementSequenceBuilder backwardRight(double inchesBackward, double inchesRight) {
        // ignore this start velocity if previous end was ignored
        boolean ignoreStartVelocity = !movements.isEmpty() && movements.getLast().ignoreEndVelocity;
        // ignore this end velocity if linked to an ignored end velocity
        boolean ignoreEndVelocity = ignoreStartVelocity && movements.getLast().linkedToNext;

        movements.add(new Movement(Movement.MovementType.BACKWARD_RIGHT, inchesBackward, inchesRight, 0, 0, 0, 0, new AprilTagValues()));

        if (ignoreStartVelocity)
            setLastIgnoredStartVelocity();
        if (ignoreEndVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a timeout to this MovementSequenceBuilder.
     *
     * @param ms The wait time in milliseconds.
     */
    public MovementSequenceBuilder sleepFor(long ms) {
        movements.add(new Movement(Movement.MovementType.DELAY, 0, 0, 0, 0, 0, ms, new AprilTagValues()));
        return this;
    }

    /**
     * Appends a rest elbow event to this MovementSequenceBuilder.
     */
    public MovementSequenceBuilder restElbow() {
        movements.add(new Movement(Movement.MovementType.REST_ELBOW, 0, 0, 0, 0, 0, 0, new AprilTagValues()));
        return this;
    }

    /**
     * Appends a flip elbow event to this MovementSequenceBuilder.
     */
    public MovementSequenceBuilder flipElbow() {
        movements.add(new Movement(Movement.MovementType.FLIP_ELBOW, 0, 0, 0, 0, 0, 0, new AprilTagValues()));
        return this;
    }

    /**
     * Appends a set wrist event to this MovementSequenceBuilder.
     */
    public MovementSequenceBuilder setWrist(double servoPosition) {
        movements.add(new Movement(Movement.MovementType.SET_WRIST, 0, 0, 0, 0, servoPosition, 0, new AprilTagValues()));
        return this;
    }

    /**
     * Appends an open claw event for both claws to this MovementSequenceBuilder.
     */
    public MovementSequenceBuilder openBothClaws() {
        return this.openLeftClaw().openRightClaw();
    }

    /**
     * Appends a close claw event for both claws to this MovementSequenceBuilder.
     */
    public MovementSequenceBuilder closeBothClaws() {
        return this.closeLeftClaw().closeRightClaw();
    }

    /**
     * Appends an open left claw event for both claws to this
     * MovementSequenceBuilder.
     */
    public MovementSequenceBuilder openLeftClaw() {
        movements.add(new Movement(Movement.MovementType.OPEN_LEFT_CLAW, 0, 0, 0, 0, 0, 0, new AprilTagValues()));
        return this;
    }

    /**
     * Appends an open right claw event for both claws to this
     * MovementSequenceBuilder.
     */
    public MovementSequenceBuilder openRightClaw() {
        movements.add(new Movement(Movement.MovementType.OPEN_RIGHT_CLAW, 0, 0, 0, 0, 0, 0, new AprilTagValues()));
        return this;
    }

    /**
     * Appends a close left claw event for both claws to this
     * MovementSequenceBuilder.
     */
    public MovementSequenceBuilder closeLeftClaw() {
        movements.add(new Movement(Movement.MovementType.CLOSE_LEFT_CLAW, 0, 0, 0, 0, 0, 0, new AprilTagValues()));
        return this;
    }

    /**
     * Appends a close right claw event for both claws to this
     * MovementSequenceBuilder.
     */
    public MovementSequenceBuilder closeRightClaw() {
        movements.add(new Movement(Movement.MovementType.CLOSE_RIGHT_CLAW, 0, 0, 0, 0, 0, 0, new AprilTagValues()));
        return this;
    }

    /**
     * Appends a lower arm event to this MovementSequenceBuilder.
     *
     * @param degrees The angle for the arm to be elevated in degrees.
     */
    public MovementSequenceBuilder lowerArm(double degrees) {
        // ignore this start velocity if previous end was ignored
        boolean ignoreStartVelocity = !movements.isEmpty() && movements.getLast().ignoreEndVelocity;
        // ignore this end velocity if linked to an ignored end velocity
        boolean ignoreEndVelocity = ignoreStartVelocity && movements.getLast().linkedToNext;

        movements.add(new Movement(Movement.MovementType.LOWER_ARM, 0, 0, 0, degrees, 0, 0, new AprilTagValues()));

        if (ignoreStartVelocity)
            setLastIgnoredStartVelocity();
        if (ignoreEndVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a raise arm event to this MovementSequenceBuilder.
     *
     * @param degrees The angle for the arm to be elevated in degrees.
     */
    public MovementSequenceBuilder raiseArm(double degrees) {
        // ignore this start velocity if previous end was ignored
        boolean ignoreStartVelocity = !movements.isEmpty() && movements.getLast().ignoreEndVelocity;
        // ignore this end velocity if linked to an ignored end velocity
        boolean ignoreEndVelocity = ignoreStartVelocity && movements.getLast().linkedToNext;

        movements.add(new Movement(Movement.MovementType.RAISE_ARM, 0, 0, 0, degrees, 0, 0, new AprilTagValues()));

        if (ignoreStartVelocity)
            setLastIgnoredStartVelocity();
        if (ignoreEndVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a rotational AprilTag alignment event to this MovementSequence.
     *
     * @param tagID an int representing the id of the tag to align with.
     * @param turnOffset a double representing the target turnOffset, where positive values are the camera facing more right of the AprilTag.
     */
    public MovementSequenceBuilder alignWithAprilTagParRot(int tagID, double turnOffset) {
        movements.add(new Movement(Movement.MovementType.APRIL_TAG_ALIGN_PAR_ROT, 0, 0, 0, 0, 0, 0, new AprilTagValues(tagID, turnOffset, 0, 0)));
        return this;
    }

    /**
     * Appends a positional AprilTag alignment event to this MovementSequence.
     *
     * @param tagID an int representing the id of the tag to align with.
     * @param yOffset a double >= 0 representing the targeted yOffset compared to the AprilTag, which positive values are "inwards".
     * @param xOffset a double representing the targeted xOffset compared to the AprilTag, where positive values have robot more right of the AprilTag.
     */
    public MovementSequenceBuilder alignWithAprilTagPos(int tagID, double yOffset, double xOffset, double turnOffset) {
        movements.add(new Movement(Movement.MovementType.APRIL_TAG_ALIGN_POS, 0, 0, 0, 0, 0, 0, new AprilTagValues(tagID, turnOffset, xOffset, yOffset)));
        return this;
    }

    ////////////////////////////////////
    // METHOD OVERLOADING FOR LINKING //
    ////////////////////////////////////

    /**
     * Appends a forward movement to this MovementSequenceBuilder.
     *
     * @param inches       How far the robot should move in inches.
     * @param withPrevious If this movement should be executed at the same time as
     *                     the previous movement is executed.
     */
    public MovementSequenceBuilder forward(double inches, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.forward(inches);
    }

    /**
     * Appends a backward movement to this MovementSequenceBuilder.
     *
     * @param inches       How far the robot should move in inches.
     * @param withPrevious If this movement should be executed at the same time as
     *                     the previous movement is executed.
     */
    public MovementSequenceBuilder backward(double inches, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.backward(inches);
    }

    /**
     * Appends a left strafe to this MovementSequenceBuilder.
     *
     * @param inches       How far the robot should strafe in inches.
     * @param withPrevious If this movement should be executed at the same time as
     *                     the previous movement is executed.
     */
    public MovementSequenceBuilder left(double inches, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.left(inches);
    }

    /**
     * Appends a right strafe to this MovementSequenceBuilder.
     *
     * @param inches       How far the robot should strafe in inches.
     * @param withPrevious If this movement should be executed at the same time as
     *                     the previous movement is executed.
     */
    public MovementSequenceBuilder right(double inches, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.right(inches);
    }

    /**
     * Appends a left turn to this MovementSequenceBuilder.
     *
     * @param degrees      How much the robot should turn in degrees.
     * @param withPrevious If this movement should be executed at the same time as
     *                     the previous movement is executed.
     */
    public MovementSequenceBuilder turnLeft(double degrees, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.turnLeft(degrees);
    }

    /**
     * Appends a left turn to this MovementSequenceBuilder.
     *
     * @param degrees      How much the robot should turn in degrees.
     * @param withPrevious If this movement should be executed at the same time as
     *                     the previous movement is executed.
     */
    public MovementSequenceBuilder turnRight(double degrees, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.turnRight(degrees);
    }

    /**
     * Appends a forward-left movement to this MovementSequenceBuilder.
     *
     * @param inchesForward How much the robot should move forward in inches.
     * @param inchesLeft    How much the robot should strafe left in inches.
     * @param withPrevious  If this movement should be executed at the same time as
     *                      the previous movement is executed.
     */
    public MovementSequenceBuilder forwardLeft(double inchesForward, double inchesLeft, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.forwardLeft(inchesForward, inchesLeft);
    }

    /**
     * Appends a forward-right movement to this MovementSequenceBuilder.
     *
     * @param inchesForward How much the robot should move forward in inches.
     * @param inchesRight   How much the robot should strafe right in inches.
     * @param withPrevious  If this movement should be executed at the same time as
     *                      the previous movement is executed.
     */
    public MovementSequenceBuilder forwardRight(double inchesForward, double inchesRight, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.forwardRight(inchesForward, inchesRight);
    }

    /**
     * Appends a backward-left movement to this MovementSequenceBuilder.
     *
     * @param inchesBackward How much the robot should move backward in inches.
     * @param inchesLeft     How much the robot should strafe left in inches.
     * @param withPrevious   If this movement should be executed at the same time as
     *                       the previous movement is executed.
     */
    public MovementSequenceBuilder backwardLeft(double inchesBackward, double inchesLeft, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.backwardLeft(inchesBackward, inchesLeft);
    }

    /**
     * Appends a backward-right movement to this MovementSequenceBuilder.
     *
     * @param inchesBackward How much the robot should move backward in inches.
     * @param inchesRight    How much the robot should strafe right in inches.
     * @param withPrevious   If this movement should be executed at the same time as
     *                       the previous movement is executed.
     */
    public MovementSequenceBuilder backwardRight(double inchesBackward, double inchesRight, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.backwardRight(inchesBackward, inchesRight);
    }

    /**
     * Appends a lower arm event to this MovementSequenceBuilder.
     *
     * @param degrees      The angle for the arm to be elevated in degrees.
     * @param withPrevious If this movement should be executed at the same time as
     *                     the previous movement is executed.
     */
    public MovementSequenceBuilder lowerArm(double degrees, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.lowerArm(degrees);
    }

    /**
     * Appends a raise arm event to this MovementSequenceBuilder.
     *
     * @param degrees      The angle for the arm to be elevated in degrees.
     * @param withPrevious If this movement should be executed at the same time as
     *                     the previous movement is executed.
     */
    public MovementSequenceBuilder raiseArm(double degrees, boolean withPrevious) {
        if (!movements.isEmpty() && withPrevious)
            linkLastMovement();
        return this.raiseArm(degrees);
    }

    //////////////////////////////////////////////
    // METHOD OVERLOADING FOR IGNORING VELOCITY //
    //////////////////////////////////////////////

    /**
     * Appends a forward movement to this MovementSequenceBuilder.
     *
     * @param inches         How far the robot should move in inches.
     * @param withPrevious   If this movement should be executed at the same time as
     *                       the previous movement is executed.
     * @param ignoreVelocity If the robot should ignore fully stopping after the
     *                       drive movement has been completed.
     */
    public MovementSequenceBuilder forward(double inches, boolean withPrevious, boolean ignoreVelocity) {
        this.forward(inches, withPrevious);

        if (ignoreVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a backward movement to this MovementSequenceBuilder.
     *
     * @param inches         How far the robot should move in inches.
     * @param withPrevious   If this movement should be executed at the same time as
     *                       the previous movement is executed.
     * @param ignoreVelocity If the robot should ignore fully stopping after the
     *                       drive movement has been completed.
     */
    public MovementSequenceBuilder backward(double inches, boolean withPrevious, boolean ignoreVelocity) {
        this.backward(inches, withPrevious);

        if (ignoreVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a left strafe to this MovementSequenceBuilder.
     *
     * @param inches         How far the robot should strafe in inches.
     * @param withPrevious   If this movement should be executed at the same time as
     *                       the previous movement is executed.
     * @param ignoreVelocity If the robot should ignore fully stopping after the
     *                       drive movement has been completed.
     */
    public MovementSequenceBuilder left(double inches, boolean withPrevious, boolean ignoreVelocity) {
        this.left(inches, withPrevious);

        if (ignoreVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a right strafe to this MovementSequenceBuilder.
     *
     * @param inches         How far the robot should strafe in inches.
     * @param withPrevious   If this movement should be executed at the same time as
     *                       the previous movement is executed.
     * @param ignoreVelocity If the robot should ignore fully stopping after the
     *                       drive movement has been completed.
     */
    public MovementSequenceBuilder right(double inches, boolean withPrevious, boolean ignoreVelocity) {
        this.right(inches, withPrevious);

        if (ignoreVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a forward-left movement to this MovementSequenceBuilder.
     *
     * @param inchesForward  How much the robot should move forward in inches.
     * @param inchesLeft     How much the robot should strafe left in inches.
     * @param withPrevious   If this movement should be executed at the same time as
     *                       the previous movement is executed.
     * @param ignoreVelocity If the robot should ignore fully stopping after the
     *                       drive movement has been completed.
     */
    public MovementSequenceBuilder forwardLeft(double inchesForward, double inchesLeft, boolean withPrevious,
                                               boolean ignoreVelocity) {
        this.forwardLeft(inchesForward, inchesLeft, withPrevious);

        if (ignoreVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a forward-right movement to this MovementSequenceBuilder.
     *
     * @param inchesForward  How much the robot should move forward in inches.
     * @param inchesRight    How much the robot should strafe right in inches.
     * @param withPrevious   If this movement should be executed at the same time as
     *                       the previous movement is executed.
     * @param ignoreVelocity If the robot should ignore fully stopping after the
     *                       drive movement has been completed.
     */
    public MovementSequenceBuilder forwardRight(double inchesForward, double inchesRight, boolean withPrevious,
                                                boolean ignoreVelocity) {
        this.forwardRight(inchesForward, inchesRight, withPrevious);

        if (ignoreVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a backward-left movement to this MovementSequenceBuilder.
     *
     * @param inchesBackward How much the robot should move backward in inches.
     * @param inchesLeft     How much the robot should strafe left in inches.
     * @param withPrevious   If this movement should be executed at the same time as
     *                       the previous movement is executed.
     * @param ignoreVelocity If the robot should ignore fully stopping after the
     *                       drive movement has been completed.
     */
    public MovementSequenceBuilder backwardLeft(double inchesBackward, double inchesLeft, boolean withPrevious,
                                                boolean ignoreVelocity) {
        this.backwardLeft(inchesBackward, inchesLeft, withPrevious);

        if (ignoreVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

    /**
     * Appends a backward-right movement to this MovementSequenceBuilder.
     *
     * @param inchesBackward How much the robot should move backward in inches.
     * @param inchesRight    How much the robot should strafe right in inches.
     * @param withPrevious   If this movement should be executed at the same time as
     *                       the previous movement is executed.
     * @param ignoreVelocity If the robot should ignore fully stopping after the
     *                       drive movement has been completed.
     */
    public MovementSequenceBuilder backwardRight(double inchesBackward, double inchesRight, boolean withPrevious,
                                                 boolean ignoreVelocity) {
        this.backwardRight(inchesBackward, inchesRight, withPrevious);

        if (ignoreVelocity)
            setLastIgnoredEndVelocity();

        return this;
    }

}
