package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleFunction;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private boolean modifiedIntegral, modifiedOutput;
    private double minIntegral;
    private double maxIntegral;
    private double minOutput;
    private double maxOutput;
    private double setPoint;
    private double integralSum;
    private double lastError;
    private double errorTolerance;
    private double VELOCITY_SPREAD_PROPORTION;

    private ElapsedTime timer;

    public PIDController() {
        this(0, 0, 0, 0);
    }

    public PIDController(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0);
    }

    public PIDController(double Kp, double Ki, double Kd, double setPoint) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.modifiedIntegral = false;
        this.minIntegral = Double.MIN_VALUE;
        this.maxIntegral = Double.MAX_VALUE;

        this.modifiedOutput = false;
        this.minOutput = Double.MIN_VALUE;
        this.maxOutput = Double.MAX_VALUE;

        this.setPoint = setPoint;

        this.integralSum = 0;
        this.lastError = 0;

        this.errorTolerance = 0;

        this.VELOCITY_SPREAD_PROPORTION = 1;

        this.timer = new ElapsedTime();
    }

    /**
     * Calculates the control value, u(t).
     * @param error The current error of the process variable.
     * @return the value produced by u(t).
     */
    public double updateError(double error) {
        return update(setPoint-error);
    }

    /**
     * Calculates the control value, u(t).
     * @param pv The current measurement of the process variable.
     * @return the value produced by u(t).
     */
    public double update(double pv) {
        // Calculate the error
        double error = setPoint - pv;

        // Calculate derivative
        double derivative = (error - lastError) / timer.seconds(); // secant line slope

        // Calculate integral
        integralSum += error * timer.seconds(); // riemann sum rectangle
        if (modifiedIntegral) {
            integralSum = Math.min(maxIntegral, integralSum);
            integralSum = Math.max(minIntegral, integralSum);
        }

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        if (modifiedOutput) {
            output = Math.min(maxOutput, output);
            output = Math.max(minOutput, output);
        }

        // Update stuff for derivative
        lastError = error;
        timer.reset();

        return output; // Return multiplier
    }

    public void setPID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void resetIntegral() {
        this.integralSum = 0;
    }

    public void setIntegralBounds(double lowerBound, double higherBound) {
        minIntegral = lowerBound;
        maxIntegral = higherBound;
        modifiedIntegral = true;
    }

    public void setOutputBounds(double lowerBound, double higherBound) {
        minOutput = lowerBound;
        maxOutput = higherBound;
        modifiedOutput = false;
    }

    public double getIntegralSum() {
        return integralSum;
    }

    public double getSetPoint() {
        return this.setPoint;
    }

    public boolean atSetPoint(double pv) {
        return Math.abs(pv-setPoint) < errorTolerance;
    }

    public double getAbsoluteDiff(double pv) {
        return Math.abs(pv-setPoint);
    }

    public void setErrorTolerance(double errorTolerance) {
        this.errorTolerance = errorTolerance;
    }

    public void setVelocitySpreadProportion(double velocitySpreadProportion) {
        this.VELOCITY_SPREAD_PROPORTION = velocitySpreadProportion;
    }

    /**
     * Calculates how long a drive movement will take.
     */
    public double[] calculateTravelTimes(double distance, double maxVelocity, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // returns {firstHalfTime, secondHalfTime}
        // maxVelocity is V
        double spread = VELOCITY_SPREAD_PROPORTION; // this is s
        double totalDist = distance; // this is D

        // calculate time for each half of the distance travelled
        double firstHalfTime, // this is FH
                secondHalfTime; // this is SH

        // calculate time for the first half
        if (ignoreStartVelocity) {

            // the velocity vs time graph should look like this:
            //
            //         V
            //         ^
            //     MAX -__________________
            //         |                 |
            //         |                 |
            //         |                 |
            //         |       D/2       |
            //         |                 |
            //         |                 |
            //        -|-----------------|-----------> t
            //         0                FH
            //
            //  using area:
            //      FH * MAX = D/2
            //      FH = D/(2*MAX)

            firstHalfTime = totalDist / (2 * maxVelocity);
        }
        else {

            // the velocity vs time graph should look like this:
            //
            //         V
            //         ^
            //     MAX -            ______
            //         |          / .    |
            //         |        /   .    |
            //         |      /     .    |
            //         |    /     D/2    |
            //         |  /         .    |
            //         |/           .    |
            //        -|------------|----|-----------> t
            //         0           sFH  FH
            //
            //  using area:
            //      1/2 * [FH + (FH-sFH)] * MAX = D/2
            //      FH * (2-s) * MAX = D
            //                D
            //      FH = -----------
            //            (2-s)*MAX

            firstHalfTime = totalDist / ((2 - spread) * maxVelocity);
        }

        // calculate time for the second half
        if (ignoreEndVelocity) {

            // the velocity vs time graph should look like this:
            //
            //         V
            //         ^
            //     MAX -__________________
            //         |                 |
            //         |                 |
            //         |                 |
            //         |       D/2       |
            //         |                 |
            //         |                 |
            //        -|-----------------|-----------> t
            //         0                SH
            //
            //  using area:
            //      SH * MAX = D/2
            //      SH = D/(2*MAX)

            secondHalfTime = totalDist / (2 * maxVelocity);
        }
        else {

            // the velocity vs time graph should look like this:
            //
            //         V
            //         ^
            //     MAX -_____
            //         |    . \
            //         |    .   \
            //         |    .     \
            //         |    D/2     \
            //         |    .         \
            //         |    .           \
            //        -|----|------------|---> t
            //         0  (1-s)SH       SH
            //
            //  using area:
            //      1/2 * [SH + (1-s)SH] * MAX = D/2
            //      SH * (2-s) * MAX = D
            //                D
            //      SH = -----------
            //            (2-s)*MAX

            secondHalfTime = totalDist / ((2 - spread) * maxVelocity);
        }

        return new double[] { firstHalfTime, secondHalfTime };
    }

    /**
     * Calculates the target velocities for the leftBack and rightBack wheels in a movement at a certain time.
     */
    public double[] calculateVelocitySetpoints(double driveSpeed, double theta, double elapsedTime, double[] travelTimes, double maxVelocity, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // this method returns {L_v, R_v}

        double spread = VELOCITY_SPREAD_PROPORTION; // this is s
        double firstHalfTime = travelTimes[0]; // time needed to travel the first half of the distance
        double secondHalfTime = travelTimes[1]; // time needed to travel the second half of the distance
        double totalTime = firstHalfTime + secondHalfTime;

        double LDir = Math.sin(theta - Math.PI / 4), RDir = Math.sin(theta + Math.PI / 4);
        double L_v, R_v; // LB and RB

        // in first half
        if (elapsedTime < firstHalfTime) {
            // rectangular graph
            if (ignoreStartVelocity) {
                L_v = (maxVelocity) * driveSpeed * LDir;
                R_v = (maxVelocity) * driveSpeed * RDir;
            }
            // trapezoidal graph
            else {
                double a = maxVelocity / (spread * firstHalfTime); // acceleration
                if (elapsedTime < spread * firstHalfTime) { // in slope
                    // y = at
                    L_v = (a * elapsedTime) * driveSpeed * LDir;
                    R_v = (a * elapsedTime) * driveSpeed * RDir;
                } else { // in plateau
                    L_v = (maxVelocity) * driveSpeed * LDir;
                    R_v = (maxVelocity) * driveSpeed * RDir;
                }
            }
        }
        // in second half
        else {
            // rectangular graph
            if (ignoreEndVelocity) {
                L_v = (maxVelocity) * driveSpeed * LDir;
                R_v = (maxVelocity) * driveSpeed * RDir;
            }
            // trapezoidal graph
            else {
                double a = maxVelocity / (spread * secondHalfTime); // acceleration
                if (elapsedTime < firstHalfTime + (1 - spread) * secondHalfTime) { // in plateau
                    L_v = (maxVelocity) * driveSpeed * LDir;
                    R_v = (maxVelocity) * driveSpeed * RDir;
                } else { // in slope
                    // y = a(T-t)
                    L_v = (a * (totalTime - elapsedTime)) * driveSpeed * LDir;
                    R_v = (a * (totalTime - elapsedTime)) * driveSpeed * RDir;
                }
            }
        }

        return new double[] { L_v, R_v };
    }

    /**
     * Calculates the target velocity for the motor in a movement at a certain time.
     */
    public double calculateVelocitySetpoints(double speed, double elapsedTime, double[] travelTimes, double maxVelocity, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // this method returns v

        double spread = VELOCITY_SPREAD_PROPORTION; // this is s
        double firstHalfTime = travelTimes[0]; // time needed to travel the first half of the distance
        double secondHalfTime = travelTimes[1]; // time needed to travel the second half of the distance
        double totalTime = firstHalfTime + secondHalfTime;

        double v;

        // in first half
        if (elapsedTime < firstHalfTime) {
            // rectangular graph
            if (ignoreStartVelocity) {
                v = (maxVelocity) * speed;
            }
            // trapezoidal graph
            else {
                double a = maxVelocity / (spread * firstHalfTime); // acceleration
                if (elapsedTime < spread * firstHalfTime) { // in slope
                    // y = at
                    v = (a * elapsedTime) * speed;
                } else { // in plateau
                    v = (maxVelocity) * speed;
                }
            }
        }
        // in second half
        else {
            // rectangular graph
            if (ignoreEndVelocity) {
                v = (maxVelocity) * speed;
            }
            // trapezoidal graph
            else {
                double a = maxVelocity / (spread * secondHalfTime); // acceleration
                if (elapsedTime < firstHalfTime + (1 - spread) * secondHalfTime) { // in plateau
                    v = (maxVelocity) * speed;
                } else { // in slope
                    // y = a(T-t)
                    v = (a * (totalTime - elapsedTime)) * speed;
                }
            }
        }

        return v;
    }

    /**
     * Calculates the target positions for the leftBack and rightBack wheels in a movement at a certain time.
     */
    public double[] calculatePositionSetpoints(double L_start, double R_start, double theta, double elapsedTime, double[] travelTimes, double maxVelocity, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // this method returns {L_sp, R_sp}

        double spread = VELOCITY_SPREAD_PROPORTION; // this is s
        double firstHalfTime = travelTimes[0]; // time needed to travel the first half of the distance
        double secondHalfTime = travelTimes[1]; // time needed to travel the second half of the distance

        double currentDistance = 0; // distance travelled to be calculated

        // five temporal waypoints:
        // W_0) 0
        // W_1) spread * firstHalfTime
        // W_2) firstHalfTime
        // W_3) firstHalfTime + (1 - spread) * secondHalfTime
        // W_4) firstHalfTime + secondHalfTime
        //
        //         V
        //         ^
        //     MAX -            ___________
        //         |          / .    .    . \
        //         |        /   .    .    .   \
        //         |      /     .    .    .     \
        //         |    /       .    .    .       \
        //         |  /         .    .    .         \
        //         |/           .    .    .           \
        //        -|------------|----|-----------------|-------> t
        //         W_0         W_1  W_2  W_3          W_4
        //
        double[] waypoints = new double[] {
                0,
                spread * firstHalfTime,
                firstHalfTime,
                firstHalfTime + (1 - spread) * secondHalfTime,
                firstHalfTime + secondHalfTime
        };
        double waypointTime;

        // calculate distance travelled
        // waypoint 1
        waypointTime = Math.max(elapsedTime - waypoints[0], 0); // elapsed non-negative time since waypoint 1
        if (elapsedTime >= waypoints[1]) // clip time to waypoint 1
            waypointTime = waypoints[1] - waypoints[0];
        // rectangular graph
        if (ignoreStartVelocity) {
            currentDistance += waypointTime * maxVelocity;
        }
        // trapezoidal graph
        else {
            double a = maxVelocity / waypoints[1]; // acceleration
            currentDistance += 0.5 * a * Math.pow(waypointTime, 2);
        }

        // waypoint 2
        waypointTime = Math.max(elapsedTime - waypoints[1], 0); // elapsed non-negative time since waypoint 2
        if (elapsedTime >= waypoints[2]) // clip time to waypoint 2
            waypointTime = waypoints[2] - waypoints[1];
        // rectangular graph
        currentDistance += waypointTime * maxVelocity;

        // waypoint 3
        waypointTime = Math.max(elapsedTime - waypoints[2], 0); // elapsed non-negative time since waypoint 3
        if (elapsedTime >= waypoints[3]) // clip time to waypoint 3
            waypointTime = waypoints[3] - waypoints[2];
        // rectangular graph
        currentDistance += waypointTime * maxVelocity;

        // waypoint 4
        waypointTime = Math.max(elapsedTime - waypoints[3], 0); // elapsed non-negative time since waypoint 4
        if (elapsedTime >= waypoints[4]) // clip time to waypoint 4
            waypointTime = waypoints[4] - waypoints[3];
        // rectangular graph
        if (ignoreEndVelocity) {
            currentDistance += waypointTime * maxVelocity;
        }
        // trapezoidal graph
        else {
            double a = maxVelocity / waypoints[1]; // acceleration
            currentDistance += 0.5 * (waypoints[4] - waypoints[3]) * (maxVelocity) - 0.5 * a * Math.pow(waypoints[4] - waypoints[3] - waypointTime, 2);
        }

        // calculate setpoints
        // LB
        double L_sp = L_start + currentDistance * Math.sin(theta - Math.PI / 4);
        // RB
        double R_sp = R_start + currentDistance * Math.sin(theta + Math.PI / 4);

        return new double[] { L_sp, R_sp };
    }

    /**
     * Calculates the target position for the motor in a movement at a certain time.
     */
    public double calculatePositionSetpoints(double start, double elapsedTime, double[] travelTimes, double maxVelocity, boolean ignoreStartVelocity, boolean ignoreEndVelocity) {

        // this method returns sp

        double spread = VELOCITY_SPREAD_PROPORTION; // this is s
        double firstHalfTime = travelTimes[0]; // time needed to travel the first half of the distance
        double secondHalfTime = travelTimes[1]; // time needed to travel the second half of the distance

        double currentDistance = 0; // distance travelled to be calculated

        // five temporal waypoints:
        // W_0) 0
        // W_1) spread * firstHalfTime
        // W_2) firstHalfTime
        // W_3) firstHalfTime + (1 - spread) * secondHalfTime
        // W_4) firstHalfTime + secondHalfTime
        //
        //         V
        //         ^
        //     MAX -            ___________
        //         |          / .    .    . \
        //         |        /   .    .    .   \
        //         |      /     .    .    .     \
        //         |    /       .    .    .       \
        //         |  /         .    .    .         \
        //         |/           .    .    .           \
        //        -|------------|----|-----------------|-------> t
        //         W_0         W_1  W_2  W_3          W_4
        //
        double[] waypoints = new double[] {
                0,
                spread * firstHalfTime,
                firstHalfTime,
                firstHalfTime + (1 - spread) * secondHalfTime,
                firstHalfTime + secondHalfTime
        };
        double waypointTime;

        // calculate distance travelled
        // waypoint 1
        waypointTime = Math.max(elapsedTime - waypoints[0], 0); // elapsed non-negative time since waypoint 1
        if (elapsedTime >= waypoints[1]) // clip time to waypoint 1
            waypointTime = waypoints[1] - waypoints[0];
        // rectangular graph
        if (ignoreStartVelocity) {
            currentDistance += waypointTime * maxVelocity;
        }
        // trapezoidal graph
        else {
            double a = maxVelocity / waypoints[1]; // acceleration
            currentDistance += 0.5 * a * Math.pow(waypointTime, 2);
        }

        // waypoint 2
        waypointTime = Math.max(elapsedTime - waypoints[1], 0); // elapsed non-negative time since waypoint 2
        if (elapsedTime >= waypoints[2]) // clip time to waypoint 2
            waypointTime = waypoints[2] - waypoints[1];
        // rectangular graph
        currentDistance += waypointTime * maxVelocity;

        // waypoint 3
        waypointTime = Math.max(elapsedTime - waypoints[2], 0); // elapsed non-negative time since waypoint 3
        if (elapsedTime >= waypoints[3]) // clip time to waypoint 3
            waypointTime = waypoints[3] - waypoints[2];
        // rectangular graph
        currentDistance += waypointTime * maxVelocity;

        // waypoint 4
        waypointTime = Math.max(elapsedTime - waypoints[3], 0); // elapsed non-negative time since waypoint 4
        if (elapsedTime >= waypoints[4]) // clip time to waypoint 4
            waypointTime = waypoints[4] - waypoints[3];
        // rectangular graph
        if (ignoreEndVelocity) {
            currentDistance += waypointTime * maxVelocity;
        }
        // trapezoidal graph
        else {
            double a = maxVelocity / waypoints[1]; // acceleration
            currentDistance += 0.5 * (waypoints[4] - waypoints[3]) * (maxVelocity) - 0.5 * a * Math.pow(waypoints[4] - waypoints[3] - waypointTime, 2);
        }

        // calculate final setpoint
        double sp = start + currentDistance;

        return sp;
    }

}