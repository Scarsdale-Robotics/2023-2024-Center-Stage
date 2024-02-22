package org.firstinspires.ftc.teamcode.subsystems.movement;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class MovementStringInterpreter {

	/**
	 * Converts a string into a MovementSequenceBuilder
	 *
	 * @param s The string to be converted.
	 */
	public static MovementSequenceBuilder toMovementSequenceBuilder(String s) {
		// removing whitespaces
		s = s.replaceAll("\\s+", "");

		ArrayList<MovementString> movementStrings = new ArrayList<>();

		for (int i = 0; 0 <= i && i < s.length(); i++) {

			// detect methods
			if (s.charAt(i) == '.') {
				// reading the MovementString name
				int openParenthesis = s.indexOf("(", i);
				String movementStringName = s.substring(i + 1, openParenthesis);
				i = openParenthesis;

				int nests = 1;
				String parametersString = "";
				while (nests > 0 && i < s.length()) {
					i++;
					if (i == s.length())
						throw new RuntimeException("Missing )");
					nests += s.charAt(i) == '(' ? 1 : (s.charAt(i) == ')' ? -1 : 0);
					if (nests > 0)
						parametersString += s.charAt(i);
				}
				ArrayList<String> movementStringParamsList = new ArrayList<>(
						Arrays.asList(parametersString.split(",")));
				movementStringParamsList.removeAll(Arrays.asList("", null));
				String[] movementStringParams = movementStringParamsList
						.toArray(new String[movementStringParamsList.size()]);
				i++;

				// append MovementString to list
				MovementString movementString = new MovementString(movementStringName, movementStringParams);
				movementStrings.add(movementString);
			}

			// detect comments
			int nextMovement = s.indexOf(".", i);
			String comment;
			if (nextMovement != -1)
				comment = s.substring(i, nextMovement);
			else
				comment = s.substring(i);
			comment = comment.replaceAll("\\s+", "");
			if (!(comment.length() == 0 || comment.startsWith("//")))
				throw new RuntimeException("Failed to interpret movement: " + comment);
			i = nextMovement - 1;
		}

		MovementSequenceBuilder movementSequenceBuilder = new MovementSequenceBuilder();

		for (MovementString movementString : movementStrings) {
			String movementStringName = movementString.name;
			String[] movementStringParams = movementString.params;

			boolean ignoreStartVelocity = !movementSequenceBuilder.getMovements().isEmpty() &&
					movementSequenceBuilder.getMovements().getLast().ignoreEndVelocity;

			if (movementStringName.equals("openBothClaws")) {
				if (movementStringParams.length != 0)
					movementString.throwInterpretError();
				movementSequenceBuilder.openBothClaws();
				continue;
			}
			if (movementStringName.equals("closeBothClaws")) {
				if (movementStringParams.length != 0)
					movementString.throwInterpretError();
				movementSequenceBuilder.closeBothClaws();
				continue;
			}

			// check for modifier parameters
			Movement convertedMovement = movementString.toMovement();
			if (movementString.extraParams.length > 0) {
				if (movementString.extraParams[0].equals("true")) // linked modifier
					if (!movementSequenceBuilder.getMovements().isEmpty())
						movementSequenceBuilder.linkLastMovement();

				boolean ignoreEndVelocity = !movementSequenceBuilder.getMovements().isEmpty() &&
						movementSequenceBuilder.getMovements().getLast().ignoreEndVelocity &&
						movementSequenceBuilder.getMovements().getLast().linkedToNext;

				movementSequenceBuilder.append(convertedMovement);

				if (movementString.extraParams.length > 1 && movementString.extraParams[1].equals("true")) { // ignore velocity modifier
					if (!movementSequenceBuilder.getMovements().isEmpty())
						movementSequenceBuilder.setLastIgnoredEndVelocity();
				}

				if (ignoreStartVelocity)
					movementSequenceBuilder.setLastIgnoredStartVelocity();

				if (ignoreEndVelocity)
					movementSequenceBuilder.setLastIgnoredEndVelocity();

				continue;
			}

			movementSequenceBuilder.append(convertedMovement);

			if (ignoreStartVelocity)
				movementSequenceBuilder.setLastIgnoredStartVelocity();

		}

		return movementSequenceBuilder;
	}

	/**
	 * Converts a string containing MovementStrings bounded by curly brackets into a
	 * MovementSequence array.
	 *
	 * @param s The string to be converted.
	 */
	public static MovementSequence[] toMovementSequenceArray(String s) {
		ArrayList<String> movementSequenceStrings = new ArrayList<>();

		Matcher matcher = Pattern.compile("\\{((.|\\n|\\r)*?)\\}").matcher(s);
		int prevEnd = 0;
		while (matcher.find()) {
			String comment = s.substring(prevEnd, matcher.start()).replaceAll("\\s+", "");
			if (!(comment.length() == 0 || comment.startsWith("//")))
				throw new RuntimeException("Unexpected token: " + comment);
			movementSequenceStrings.add(matcher.group().replaceAll("\\{|\\}", ""));
			prevEnd = matcher.end() + 1;
		}
		String comment = s.substring(Math.min(prevEnd, s.length())).replaceAll("\\s+", "");
		if (!(comment.length() == 0 || comment.startsWith("//")))
			throw new RuntimeException("Unexpected token: " + comment);

		MovementSequence[] movementSequences = new MovementSequence[movementSequenceStrings.size()];
		for (int i = 0; i < movementSequenceStrings.size(); i++)
			movementSequences[i] = toMovementSequenceBuilder(movementSequenceStrings.get(i)).build();

		return movementSequences;
	}

	private static class MovementString {
		String name;
		String[] params;
		String[] extraParams;

		public MovementString(String name, String[] params) {
			this.name = name;
			this.params = params;
			extraParams = new String[0];
		}

		public String toString() {
			String s = name + "(";
			for (int i = 0; i < params.length; i++)
				s += params[i] + (i != params.length - 1 ? "," : "");
			s += ")";
			return s;
		}

		/**
		 * Converts a MovementString to a Movement.
		 *
		 * @return The converted Movement.
		 */
		public Movement toMovement() {

			Movement movement = null;

			switch (name) {

				case "alignWithAprilTagParRot":
					if (!isIn(params.length, 2))
						throwInterpretError();
					movement = alignWithAprilTagParRot((int)eval(params[0]), eval(params[1]));
					break;

				case "alignWithAprilTagPos":
					if (!isIn(params.length, 4))
						throwInterpretError();
					movement = alignWithAprilTagPos((int)eval(params[0]), eval(params[1]), eval(params[2]), eval(params[3]));
					break;

				case "forward":
					if (!isIn(params.length, 1, 2, 3))
						throwInterpretError();
					movement = forward(eval(params[0]));
					if (params.length > 1)
						extraParams = Arrays.copyOfRange(params, 1, params.length);
					break;

				case "backward":
					if (!isIn(params.length, 1, 2, 3))
						throwInterpretError();
					movement = backward(eval(params[0]));
					if (params.length > 1)
						extraParams = Arrays.copyOfRange(params, 1, params.length);
					break;

				case "left":
					if (!isIn(params.length, 1, 2, 3))
						throwInterpretError();
					movement = left(eval(params[0]));
					if (params.length > 1)
						extraParams = Arrays.copyOfRange(params, 1, params.length);
					break;

				case "right":
					if (!isIn(params.length, 1, 2, 3))
						throwInterpretError();
					movement = right(eval(params[0]));
					if (params.length > 1)
						extraParams = Arrays.copyOfRange(params, 1, params.length);
					break;

				case "turnLeft":
					if (!isIn(params.length, 1, 2))
						throwInterpretError();
					movement = turnLeft(eval(params[0]));
					if (params.length > 1)
						extraParams = Arrays.copyOfRange(params, 1, params.length);
					break;

				case "turnRight":
					if (!isIn(params.length, 1, 2))
						throwInterpretError();
					movement = turnRight(eval(params[0]));
					if (params.length > 1)
						extraParams = Arrays.copyOfRange(params, 1, params.length);
					break;

				case "forwardLeft":
					if (!isIn(params.length, 2, 3, 4))
						throwInterpretError();
					movement = forwardLeft(eval(params[0]), eval(params[1]));
					if (params.length > 2)
						extraParams = Arrays.copyOfRange(params, 2, params.length);
					break;

				case "forwardRight":
					if (!isIn(params.length, 2, 3, 4))
						throwInterpretError();
					movement = forwardRight(eval(params[0]), eval(params[1]));
					if (params.length > 2)
						extraParams = Arrays.copyOfRange(params, 2, params.length);
					break;

				case "backwardLeft":
					if (!isIn(params.length, 2, 3, 4))
						throwInterpretError();
					movement = backwardLeft(eval(params[0]), eval(params[1]));
					if (params.length > 2)
						extraParams = Arrays.copyOfRange(params, 2, params.length);
					break;

				case "backwardRight":
					if (!isIn(params.length, 2, 3, 4))
						throwInterpretError();
					movement = backwardRight(eval(params[0]), eval(params[1]));
					if (params.length > 2)
						extraParams = Arrays.copyOfRange(params, 2, params.length);
					break;

				case "sleepFor":
					if (!isIn(params.length, 1))
						throwInterpretError();
					movement = sleepFor((long) eval(params[0]));
					break;

				case "restElbow":
					if (!isIn(params.length, 0))
						throwInterpretError();
					movement = restElbow();
					break;

				case "flipElbow":
					if (!isIn(params.length, 0))
						throwInterpretError();
					movement = flipElbow();
					break;

				case "setWrist":
					if (!isIn(params.length, 1))
						throwInterpretError();
					movement = setWrist(eval(params[0]));
					break;

				case "openLeftClaw":
					if (!isIn(params.length, 0))
						throwInterpretError();
					movement = openLeftClaw();
					break;

				case "openRightClaw":
					if (!isIn(params.length, 0))
						throwInterpretError();
					movement = openRightClaw();
					break;

				case "closeLeftClaw":
					if (!isIn(params.length, 0))
						throwInterpretError();
					movement = closeLeftClaw();
					break;

				case "closeRightClaw":
					if (!isIn(params.length, 0))
						throwInterpretError();
					movement = closeRightClaw();
					break;

				case "lowerArm":
					if (!isIn(params.length, 1, 2))
						throwInterpretError();
					movement = lowerArm(eval(params[0]));
					if (params.length > 1)
						extraParams = Arrays.copyOfRange(params, 1, params.length);
					break;

				case "raiseArm":
					if (!isIn(params.length, 1, 2))
						throwInterpretError();
					movement = raiseArm(eval(params[0]));
					if (params.length > 1)
						extraParams = Arrays.copyOfRange(params, 1, params.length);
					break;

				default:
					throwInterpretError();

			}

			return movement;

		}

		private void throwInterpretError() {
			throw new IllegalArgumentException("Failed to interpret the movement " + this);
		}

	}

	/////////////////////////////////////////////////////
	/////////////////////////////////////////////////////
	/////////////////////////////////////////////////////
	/////////////////////////////////////////////////////
	/////////////////////////////////////////////////////

	private static Movement alignWithAprilTagParRot(int tagID, double turnOffset) {
		return (new Movement(Movement.MovementType.APRIL_TAG_ALIGN_PAR_ROT, 0, 0, 0, 0, 0, 0, new AprilTagValues(tagID, turnOffset, 0, 0)));
	}
	public static Movement alignWithAprilTagPos(int tagID, double yOffset, double xOffset, double turnOffset) {
		return (new Movement(Movement.MovementType.APRIL_TAG_ALIGN_POS, 0, 0, 0, 0, 0, 0, new AprilTagValues(tagID, turnOffset, xOffset, yOffset)));
	}

	/**
	 * Appends a forward movement to this MovementSequenceBuilder.
	 *
	 * @param inches How far the robot should move in inches.
	 */
	private static Movement forward(double inches) {
		return (new Movement(Movement.MovementType.FORWARD, inches, 0, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a backward movement to this MovementSequenceBuilder.
	 *
	 * @param inches How far the robot should move in inches.
	 */
	private static Movement backward(double inches) {
		return (new Movement(Movement.MovementType.BACKWARD, inches, 0, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a left strafe to this MovementSequenceBuilder.
	 *
	 * @param inches How far the robot should strafe in inches.
	 */
	private static Movement left(double inches) {
		return (new Movement(Movement.MovementType.STRAFE_LEFT, 0, inches, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a right strafe to this MovementSequenceBuilder.
	 *
	 * @param inches How far the robot should strafe in inches.
	 */
	private static Movement right(double inches) {
		return (new Movement(Movement.MovementType.STRAFE_RIGHT, 0, inches, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a left turn to this MovementSequenceBuilder.
	 *
	 * @param degrees How much the robot should turn in degrees.
	 */
	private static Movement turnLeft(double degrees) {
		return (new Movement(Movement.MovementType.TURN_LEFT, 0, 0, degrees, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a left turn to this MovementSequenceBuilder.
	 *
	 * @param degrees How much the robot should turn in degrees.
	 */
	private static Movement turnRight(double degrees) {
		return (new Movement(Movement.MovementType.TURN_RIGHT, 0, 0, degrees, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a forward-left movement to this MovementSequenceBuilder.
	 *
	 * @param inchesForward How much the robot should move forward in inches.
	 * @param inchesLeft    How much the robot should strafe left in inches.
	 */
	private static Movement forwardLeft(double inchesForward, double inchesLeft) {
		return (new Movement(Movement.MovementType.FORWARD_LEFT, inchesForward, inchesLeft, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a forward-right movement to this MovementSequenceBuilder.
	 *
	 * @param inchesForward How much the robot should move forward in inches.
	 * @param inchesRight   How much the robot should strafe right in inches.
	 */
	private static Movement forwardRight(double inchesForward, double inchesRight) {
		return (new Movement(Movement.MovementType.FORWARD_RIGHT, inchesForward, inchesRight, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a backward-left movement to this MovementSequenceBuilder.
	 *
	 * @param inchesBackward How much the robot should move backward in inches.
	 * @param inchesLeft     How much the robot should strafe left in inches.
	 */
	private static Movement backwardLeft(double inchesBackward, double inchesLeft) {
		return (new Movement(Movement.MovementType.BACKWARD_LEFT, inchesBackward, inchesLeft, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a backward-right movement to this MovementSequenceBuilder.
	 *
	 * @param inchesBackward How much the robot should move backward in inches.
	 * @param inchesRight    How much the robot should strafe right in inches.
	 */
	private static Movement backwardRight(double inchesBackward, double inchesRight) {
		return (new Movement(Movement.MovementType.BACKWARD_RIGHT, inchesBackward, inchesRight, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a timeout to this MovementSequenceBuilder.
	 *
	 * @param ms The wait time in milliseconds.
	 */
	private static Movement sleepFor(long ms) {
		return (new Movement(Movement.MovementType.DELAY, 0, 0, 0, 0, 0, ms, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a rest elbow event to this MovementSequenceBuilder.
	 */
	private static Movement restElbow() {
		return (new Movement(Movement.MovementType.REST_ELBOW, 0, 0, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a flip elbow event to this MovementSequenceBuilder.
	 */
	private static Movement flipElbow() {
		return (new Movement(Movement.MovementType.FLIP_ELBOW, 0, 0, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a set wrist event to this MovementSequenceBuilder.
	 */
	private static Movement setWrist(double servoPosition) {
		return (new Movement(Movement.MovementType.SET_WRIST, 0, 0, 0, 0, servoPosition, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends an open left claw event for both claws to this
	 * MovementSequenceBuilder.
	 */
	private static Movement openLeftClaw() {
		return (new Movement(Movement.MovementType.OPEN_LEFT_CLAW, 0, 0, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends an open right claw event for both claws to this
	 * MovementSequenceBuilder.
	 */
	private static Movement openRightClaw() {
		return (new Movement(Movement.MovementType.OPEN_RIGHT_CLAW, 0, 0, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a close left claw event for both claws to this
	 * MovementSequenceBuilder.
	 */
	private static Movement closeLeftClaw() {
		return (new Movement(Movement.MovementType.CLOSE_LEFT_CLAW, 0, 0, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a close right claw event for both claws to this
	 * MovementSequenceBuilder.
	 */
	private static Movement closeRightClaw() {
		return (new Movement(Movement.MovementType.CLOSE_RIGHT_CLAW, 0, 0, 0, 0, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a lower arm event to this MovementSequenceBuilder.
	 *
	 * @param degrees The angle for the arm to be elevated in degrees.
	 */
	private static Movement lowerArm(double degrees) {
		return (new Movement(Movement.MovementType.LOWER_ARM, 0, 0, 0, degrees, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Appends a raise arm event to this MovementSequenceBuilder.
	 *
	 * @param degrees The angle for the arm to be elevated in degrees.
	 */
	private static Movement raiseArm(double degrees) {
		return (new Movement(Movement.MovementType.RAISE_ARM, 0, 0, 0, degrees, 0, 0, new AprilTagValues(-1, 0, 0, 0)));

	}

	/**
	 * Checks if an integer is in a set of integer values.
	 *
	 * @param key    The integer to be matched.
	 * @param values The values to be matched with the key.
	 */
	private static boolean isIn(int key, int... values) {
		return Arrays.stream(values).anyMatch(i -> i == key);
	}

	/**
	 * See:
	 * https://stackoverflow.com/questions/3422673/how-to-evaluate-a-math-expression-given-in-string-form
	 *
	 * @param str The string expression to be evaluated.
	 * @return The answer to the expression.
	 */
	private static double eval(final String str) {
		return new Object() {
			int pos = -1, ch;

			void nextChar() {
				ch = (++pos < str.length()) ? str.charAt(pos) : -1;
			}

			boolean eat(int charToEat) {
				while (ch == ' ')
					nextChar();
				if (ch == charToEat) {
					nextChar();
					return true;
				}
				return false;
			}

			double parse() {
				nextChar();
				double x = parseExpression();
				if (pos < str.length())
					throw new RuntimeException("Unexpected: " + (char) ch);
				return x;
			}

			// Grammar:
			// expression = term | expression `+` term | expression `-` term
			// term = factor | term `*` factor | term `/` factor
			// factor = `+` factor | `-` factor | `(` expression `)` | number
			// | functionName `(` expression `)` | functionName factor
			// | factor `^` factor

			double parseExpression() {
				double x = parseTerm();
				for (;;) {
					if (eat('+'))
						x += parseTerm(); // addition
					else if (eat('-'))
						x -= parseTerm(); // subtraction
					else
						return x;
				}
			}

			double parseTerm() {
				double x = parseFactor();
				for (;;) {
					if (eat('*'))
						x *= parseFactor(); // multiplication
					else if (eat('/'))
						x /= parseFactor(); // division
					else
						return x;
				}
			}

			double parseFactor() {
				if (eat('+'))
					return +parseFactor(); // unary plus
				if (eat('-'))
					return -parseFactor(); // unary minus

				double x;
				int startPos = this.pos;
				if (eat('(')) { // parentheses
					x = parseExpression();
					if (!eat(')'))
						throw new RuntimeException("Missing ')'");
				} else if ((ch >= '0' && ch <= '9') || ch == '.') { // numbers
					while ((ch >= '0' && ch <= '9') || ch == '.')
						nextChar();
					x = Double.parseDouble(str.substring(startPos, this.pos));
				} else if (ch >= 'a' && ch <= 'z') { // functions
					while (ch >= 'a' && ch <= 'z')
						nextChar();
					String func = str.substring(startPos, this.pos);
					if (eat('(')) {
						x = parseExpression();
						if (!eat(')'))
							throw new RuntimeException("Missing ')' after argument to " + func);
					} else {
						x = parseFactor();
					}
					if (func.equals("sqrt"))
						x = Math.sqrt(x);
					else if (func.equals("sin"))
						x = Math.sin(Math.toRadians(x));
					else if (func.equals("cos"))
						x = Math.cos(Math.toRadians(x));
					else if (func.equals("tan"))
						x = Math.tan(Math.toRadians(x));
					else
						throw new RuntimeException("Failed to interpret movement: " + func);
				} else {
					throw new RuntimeException("Unexpected: " + (char) ch);
				}

				if (eat('^'))
					x = Math.pow(x, parseFactor()); // exponentiation

				return x;
			}
		}.parse();
	}

}
