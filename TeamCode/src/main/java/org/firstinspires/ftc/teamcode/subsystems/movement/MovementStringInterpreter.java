package org.firstinspires.ftc.teamcode.subsystems.movement;

import java.util.ArrayList;
import java.util.Arrays;

public class MovementStringInterpreter {

	/**
	 * Converts a string into a MovementSequenceBuilder
	 * @param s      The string to be converted.
	 */
	public static MovementSequenceBuilder interpretString(String s) {
		// removing whitespaces
		s = s.replaceAll("\\s+","");

		ArrayList<MovementString> movementStrings = new ArrayList<>();

		for (int i = 0; i < s.length(); i++) {
			if (s.charAt(i) == '.') {
				// reading the MovementString name
				int openParenthesis = s.indexOf("(", i);
				String movementStringName = s.substring(i+1, openParenthesis);
				i = openParenthesis;

				// reading the MovementString parameters
				int closeParenthesis = s.indexOf(")", i);
				int nests = 1;
				String parametersString = "";
				while (nests>0 && i < s.length()) {
					i++;
					nests += s.charAt(i)=='(' ? 1 : (s.charAt(i)==')' ? -1 : 0);
					if (nests>0)
						parametersString += s.charAt(i);
				}
				ArrayList<String> movementStringParamsList= new ArrayList<>(Arrays.asList(parametersString.split(",")));
				movementStringParamsList.removeAll(Arrays.asList("", null));
				String[] movementStringParams = movementStringParamsList.toArray(new String[movementStringParamsList.size()]);
				i = closeParenthesis;

				// append MovementString to list
				MovementString movementString = new MovementString(movementStringName, movementStringParams);
				movementStrings.add(movementString);
			}
		}

		MovementSequenceBuilder movementSequenceBuilder = new MovementSequenceBuilder();

		for (MovementString movementString : movementStrings) {
			String movementStringName = movementString.name;
			String[] movementStringParams = movementString.params;

			if (movementStringName.equals("openBothClaws")) {
				if (movementStringParams.length!=0) movementString.throwInterpretError();
				movementSequenceBuilder.openBothClaws();
				continue;
			}
			if (movementStringName.equals("closeBothClaws")) {
				if (movementStringParams.length!=0) movementString.throwInterpretError();
				movementSequenceBuilder.closeBothClaws();
				continue;
			}

			if (movementStringParams.length > 0 &&
					movementStringParams[movementStringParams.length-1].equals("true")) {
				movementSequenceBuilder.linkLastMovement();
				movementSequenceBuilder.appendMovement(movementString.toMovement());
				continue;
			}

			movementSequenceBuilder.appendMovement(movementString.toMovement());

		}

		return movementSequenceBuilder;
	}


	private static class MovementString {
		String name;
		String[] params;

		public MovementString(String name, String[] params) {
			this.name = name;
			this.params = params;
		}

		public void popLastParam() {
			params = Arrays.copyOf(params, params.length-1);
		}

		public String toString() {
			String s = name+"(";
			for (int i = 0; i < params.length; i++)
				s += params[i] + (i!=params.length-1 ? "," : "");
			s += ")";
			return s;
		}

		/**
		 * Converts a MovementString to a Movement.
		 * @return The converted Movement.
		 */
		public Movement toMovement() {

			Movement movement = null;

			switch (name) {

				case "alignWithWhitePixel":
					if (params.length!=0) throwInterpretError();
					movement = alignWithWhitePixel();
					break;

				case "forward":
					if (params.length!=1 && params.length!=2) throwInterpretError();
					movement = forward(eval(params[0]));
					break;

				case "backward":
					if (params.length!=1 && params.length!=2) throwInterpretError();
					movement = backward(eval(params[0]));
					break;

				case "left":
					if (params.length!=1 && params.length!=2) throwInterpretError();
					movement = left(eval(params[0]));
					break;

				case "right":
					if (params.length!=1 && params.length!=2) throwInterpretError();
					movement = right(eval(params[0]));
					break;

				case "turnLeft":
					if (params.length!=1 && params.length!=2) throwInterpretError();
					movement = turnLeft(eval(params[0]));
					break;

				case "turnRight":
					if (params.length!=1 && params.length!=2) throwInterpretError();
					movement = turnRight(eval(params[0]));
					break;

				case "forwardLeft":
					if (params.length!=2 && params.length!=3) throwInterpretError();
					movement = forwardLeft(eval(params[0]),
							eval(params[1]));
					break;

				case "forwardRight":
					if (params.length!=2 && params.length!=3) throwInterpretError();
					movement = forwardRight(eval(params[0]),
							eval(params[1]));
					break;

				case "backwardLeft":
					if (params.length!=2 && params.length!=3) throwInterpretError();
					movement = backwardLeft(eval(params[0]),
							eval(params[1]));
					break;

				case "backwardRight":
					if (params.length!=2 && params.length!=3) throwInterpretError();
					movement = backwardRight(eval(params[0]),
							eval(params[1]));
					break;

				case "sleepFor":
					if (params.length!=1) throwInterpretError();
					movement = sleepFor((long)eval(params[0]));
					break;

				case "restElbow":
					if (params.length!=0) throwInterpretError();
					movement = restElbow();
					break;

				case "flipElbow":
					if (params.length!=0) throwInterpretError();
					movement = flipElbow();
					break;

				case "setWrist":
					if (params.length!=1) throwInterpretError();
					movement = setWrist(eval(params[0]));
					break;

				case "openLeftClaw":
					if (params.length!=0) throwInterpretError();
					movement = openLeftClaw();
					break;

				case "openRightClaw":
					if (params.length!=0) throwInterpretError();
					movement = openRightClaw();
					break;

				case "closeLeftClaw":
					if (params.length!=0) throwInterpretError();
					movement = closeLeftClaw();
					break;

				case "closeRightClaw":
					if (params.length!=0) throwInterpretError();
					movement = closeRightClaw();
					break;

				case "lowerArm":
					if (params.length!=1 && params.length!=2) throwInterpretError();
					movement = lowerArm(eval(params[0]));
					break;

				case "raiseArm":
					if (params.length!=1 && params.length!=2) throwInterpretError();
					movement = raiseArm(eval(params[0]));
					break;

				default:
					throwInterpretError();

			}

			return movement;

		}

		private void throwInterpretError() {
			throw new IllegalArgumentException("Failed to interpret the movement "+this);
		}

	}

	/////////////////////////////////////////////////////
	/////////////////////////////////////////////////////
	/////////////////////////////////////////////////////
	/////////////////////////////////////////////////////
	/////////////////////////////////////////////////////

	public static Movement alignWithWhitePixel() {
		return (new Movement(Movement.MovementType.WHITE_PXL_ALIGN, 0, 0, 0, 0, 0, 0));

	}

	/**
	 * Appends a forward movement to this MovementSequenceBuilder.
	 * @param inches      How far the robot should move in inches.
	 */
	public static Movement forward(double inches) {
		return (new Movement(Movement.MovementType.FORWARD, inches, 0, 0, 0, 0, 0));

	}

	/**
	 * Appends a backward movement to this MovementSequenceBuilder.
	 * @param inches      How far the robot should move in inches.
	 */
	public static Movement backward(double inches) {
		return (new Movement(Movement.MovementType.BACKWARD, inches, 0, 0, 0, 0, 0));

	}

	/**
	 * Appends a left strafe to this MovementSequenceBuilder.
	 * @param inches      How far the robot should strafe in inches.
	 */
	public static Movement left(double inches) {
		return (new Movement(Movement.MovementType.STRAFE_LEFT, 0, inches, 0, 0, 0, 0));

	}

	/**
	 * Appends a right strafe to this MovementSequenceBuilder.
	 * @param inches      How far the robot should strafe in inches.
	 */
	public static Movement right(double inches) {
		return (new Movement(Movement.MovementType.STRAFE_RIGHT, 0, inches, 0, 0, 0, 0));

	}

	/**
	 * Appends a left turn to this MovementSequenceBuilder.
	 * @param degrees      How much the robot should turn in degrees.
	 */
	public static Movement turnLeft(double degrees) {
		return (new Movement(Movement.MovementType.TURN_LEFT, 0, 0, degrees, 0, 0, 0));

	}

	/**
	 * Appends a left turn to this MovementSequenceBuilder.
	 * @param degrees      How much the robot should turn in degrees.
	 */
	public static Movement turnRight(double degrees) {
		return (new Movement(Movement.MovementType.TURN_RIGHT, 0, 0, degrees, 0, 0, 0));

	}

	/**
	 * Appends a forward-left movement to this MovementSequenceBuilder.
	 * @param inchesForward      How much the robot should move forward in inches.
	 * @param inchesLeft         How much the robot should strafe left in inches.
	 */
	public static Movement forwardLeft(double inchesForward, double inchesLeft) {
		return (new Movement(Movement.MovementType.FORWARD_LEFT, inchesForward, inchesLeft, 0, 0, 0, 0));

	}

	/**
	 * Appends a forward-right movement to this MovementSequenceBuilder.
	 * @param inchesForward      How much the robot should move forward in inches.
	 * @param inchesRight        How much the robot should strafe right in inches.
	 */
	public static Movement forwardRight(double inchesForward, double inchesRight) {
		return (new Movement(Movement.MovementType.FORWARD_RIGHT, inchesForward, inchesRight, 0, 0, 0, 0));

	}

	/**
	 * Appends a backward-left movement to this MovementSequenceBuilder.
	 * @param inchesBackward     How much the robot should move backward in inches.
	 * @param inchesLeft         How much the robot should strafe left in inches.
	 */
	public static Movement backwardLeft(double inchesBackward, double inchesLeft) {
		return (new Movement(Movement.MovementType.BACKWARD_LEFT, inchesBackward, inchesLeft, 0, 0, 0, 0));

	}

	/**
	 * Appends a backward-right movement to this MovementSequenceBuilder.
	 * @param inchesBackward     How much the robot should move backward in inches.
	 * @param inchesRight        How much the robot should strafe right in inches.
	 */
	public static Movement backwardRight(double inchesBackward, double inchesRight) {
		return (new Movement(Movement.MovementType.BACKWARD_RIGHT, inchesBackward, inchesRight, 0, 0, 0, 0));

	}

	/**
	 * Appends a timeout to this MovementSequenceBuilder.
	 * @param ms      The wait time in milliseconds.
	 */
	public static Movement sleepFor(long ms) {
		return (new Movement(Movement.MovementType.DELAY, 0, 0, 0, 0, 0, ms));

	}

	/**
	 * Appends a rest elbow event to this MovementSequenceBuilder.
	 */
	public static Movement restElbow() {
		return (new Movement(Movement.MovementType.REST_ELBOW, 0, 0, 0, 0, 0, 0));

	}

	/**
	 * Appends a flip elbow event to this MovementSequenceBuilder.
	 */
	public static Movement flipElbow() {
		return (new Movement(Movement.MovementType.FLIP_ELBOW, 0, 0, 0, 0, 0, 0));

	}

	/**
	 * Appends a set wrist event to this MovementSequenceBuilder.
	 */
	public static Movement setWrist(double servoPosition) {
		return (new Movement(Movement.MovementType.SET_WRIST, 0, 0, 0, 0, servoPosition, 0));

	}

	/**
	 * Appends an open left claw event for both claws to this MovementSequenceBuilder.
	 */
	public static Movement openLeftClaw() {
		return (new Movement(Movement.MovementType.OPEN_LEFT_CLAW, 0, 0, 0, 0, 0, 0));

	}

	/**
	 * Appends an open right claw event for both claws to this MovementSequenceBuilder.
	 */
	public static Movement openRightClaw() {
		return (new Movement(Movement.MovementType.OPEN_RIGHT_CLAW, 0, 0, 0, 0, 0, 0));

	}

	/**
	 * Appends a close left claw event for both claws to this MovementSequenceBuilder.
	 */
	public static Movement closeLeftClaw() {
		return (new Movement(Movement.MovementType.CLOSE_LEFT_CLAW, 0, 0, 0, 0, 0, 0));

	}

	/**
	 * Appends a close right claw event for both claws to this MovementSequenceBuilder.
	 */
	public static Movement closeRightClaw() {
		return (new Movement(Movement.MovementType.CLOSE_RIGHT_CLAW, 0, 0, 0, 0, 0, 0));

	}

	/**
	 * Appends a lower arm event to this MovementSequenceBuilder.
	 * @param degrees      The angle for the arm to be elevated in degrees.
	 */
	public static Movement lowerArm(double degrees) {
		return (new Movement(Movement.MovementType.LOWER_ARM, 0, 0, 0, degrees, 0, 0));

	}

	/**
	 * Appends a raise arm event to this MovementSequenceBuilder.
	 * @param degrees      The angle for the arm to be elevated in degrees.
	 */
	public static Movement raiseArm(double degrees) {
		return (new Movement(Movement.MovementType.RAISE_ARM, 0, 0, 0, degrees, 0, 0));

	}

	/**
	 * See: https://stackoverflow.com/questions/3422673/how-to-evaluate-a-math-expression-given-in-string-form
	 * @param str  The string expression to be evaluated.
	 * @return The answer to the expression.
	 */
	public static double eval(final String str) {
		return new Object() {
			int pos = -1, ch;

			void nextChar() {
				ch = (++pos < str.length()) ? str.charAt(pos) : -1;
			}

			boolean eat(int charToEat) {
				while (ch == ' ') nextChar();
				if (ch == charToEat) {
					nextChar();
					return true;
				}
				return false;
			}

			double parse() {
				nextChar();
				double x = parseExpression();
				if (pos < str.length()) throw new RuntimeException("Unexpected: " + (char)ch);
				return x;
			}

			// Grammar:
			// expression = term | expression `+` term | expression `-` term
			// term = factor | term `*` factor | term `/` factor
			// factor = `+` factor | `-` factor | `(` expression `)` | number
			//        | functionName `(` expression `)` | functionName factor
			//        | factor `^` factor

			double parseExpression() {
				double x = parseTerm();
				for (;;) {
					if      (eat('+')) x += parseTerm(); // addition
					else if (eat('-')) x -= parseTerm(); // subtraction
					else return x;
				}
			}

			double parseTerm() {
				double x = parseFactor();
				for (;;) {
					if      (eat('*')) x *= parseFactor(); // multiplication
					else if (eat('/')) x /= parseFactor(); // division
					else return x;
				}
			}

			double parseFactor() {
				if (eat('+')) return +parseFactor(); // unary plus
				if (eat('-')) return -parseFactor(); // unary minus

				double x;
				int startPos = this.pos;
				if (eat('(')) { // parentheses
					x = parseExpression();
					if (!eat(')')) throw new RuntimeException("Missing ')'");
				} else if ((ch >= '0' && ch <= '9') || ch == '.') { // numbers
					while ((ch >= '0' && ch <= '9') || ch == '.') nextChar();
					x = Double.parseDouble(str.substring(startPos, this.pos));
				} else if (ch >= 'a' && ch <= 'z') { // functions
					while (ch >= 'a' && ch <= 'z') nextChar();
					String func = str.substring(startPos, this.pos);
					if (eat('(')) {
						x = parseExpression();
						if (!eat(')')) throw new RuntimeException("Missing ')' after argument to " + func);
					} else {
						x = parseFactor();
					}
					if (func.equals("sqrt")) x = Math.sqrt(x);
					else if (func.equals("sin")) x = Math.sin(Math.toRadians(x));
					else if (func.equals("cos")) x = Math.cos(Math.toRadians(x));
					else if (func.equals("tan")) x = Math.tan(Math.toRadians(x));
					else throw new RuntimeException("Failed to interpret movement: " + func);
				} else {
					throw new RuntimeException("Unexpected: " + (char)ch);
				}

				if (eat('^')) x = Math.pow(x, parseFactor()); // exponentiation

				return x;
			}
		}.parse();
	}

}
