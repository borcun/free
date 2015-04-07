import java.util.*;

/** Class that can evaluate a postfix expression.
*   @author Koffman & Wolfgang
* */

public class PostfixEvaluator 
{
  // Constant
  /** A list of operators. */
  private static final String OPERATORS = "+-*/";

  // Data Field
  /** The operand stack. */
  private Stack < Integer > operandStack;

  // Methods
  /** Evaluates the current operation.
      This function pops the two operands off the operand
      stack and applies the operator.
      @param op A character representing the operator
      @return The result of applying the operator
 * @throws Exception 
      @throws EmptyStackException if pop is attempted on
              an empty stack
   */
  private int evalOp(char op, int lineNumber) throws Exception {
    // Pop the two operands off the stack.
    int rhs = operandStack.pop();
    int lhs = operandStack.pop();
    int result = 0;
    // Evaluate the operator.
    switch (op) {
      case '+':
        result = lhs + rhs;
        break;
      case '-':
        result = lhs - rhs;
        break;
      case '/':
    	if(rhs == 0)
    		throw new Exception(Error.divisionByZero(lineNumber));
    		result = lhs / rhs;
        break;
      case '*':
        result = lhs * rhs;
        break;

    }
    return result;
  }

  /** Determines whether a character is an operator.
      @param op The character to be tested
      @return true if the character is an operator
   */
  private boolean isOperator(char ch) {
    return OPERATORS.indexOf(ch) != -1;
  }

  /** Evaluates a postfix expression.
      @param expression The expression to be evaluated
      @return The value of the expression
      @throws SyntaxErrorException if a syntax error is detected
   */
  public int eval(String expression, int lineNumber) throws Exception {
    // Create an empty stack.
    operandStack = new Stack < Integer > ();

    // Process each token.
    StringTokenizer tokens = new StringTokenizer(expression);
    try {
      while (tokens.hasMoreTokens()) {
        String nextToken = tokens.nextToken();
        // Does it start with a digit?
        if (Character.isDigit(nextToken.charAt(0))) {
          // Get the integer value.
          int value = Integer.parseInt(nextToken);
          // Push value onto operand stack.
          operandStack.push(value);
        } // Is it an operator?
        else if (isOperator(nextToken.charAt(0))) {
          // Evaluate the operator.
          int result = evalOp(nextToken.charAt(0),lineNumber);
          // Push result onto the operand stack.
          operandStack.push(result);
        }
        else {
          // Invalid character.
          throw new Exception(Error.unExceptedChar(lineNumber));
        }
      } // End while.

      // No more tokens - pop result from operand stack.
      int answer = operandStack.pop();
      // Operand stack should be empty.
      if (operandStack.empty()) {
        return answer;
      }
      else {
        // Indicate syntax error.
        throw new Exception(Error.emptyStack(lineNumber));
      }
    }
    catch (EmptyStackException ex) {
      // Pop was attempted on an empty stack.
      throw new Exception(Error.emptyStack(lineNumber));
    }
  }
}
