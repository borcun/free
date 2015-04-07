import java.util.*;

/** Translates an infix expression with parentheses
*   to a postfix expression.
*   @author Koffman & Wolfgang
*/

public class InfixToPostfixParens 
{
  // Data Fields
  /** The operator stack */
  private Stack < Character > operatorStack;

  /** The operators */
  private static final String OPERATORS = "+-*/()";

  /** The precedence of the operators, matches order of OPERATORS. */
  private static final int[] PRECEDENCE = {1, 1, 2, 2, -1, -1};

  /** The postfix string */
  private StringBuilder postfix;

  /** Convert a string from infix to postfix.
      @param infix The infix expression
      @throws SyntaxErrorException
   */
  public String convert(String infix, int lineNumber) throws Exception {
    operatorStack = new Stack < Character > ();
    postfix = new StringBuilder();
    StringTokenizer infixTokens = new StringTokenizer(infix);
    try {
      // Process each token in the infix string.
      while (infixTokens.hasMoreTokens()) {
        String nextToken = infixTokens.nextToken();
        char firstChar = nextToken.charAt(0);
        // Is it an operand?
        if (Character.isJavaIdentifierStart(firstChar) || Character.isDigit(firstChar)) {
          postfix.append(nextToken);
          postfix.append(' ');
        } // Is it an operator?
        else if (isOperator(firstChar)) {
          processOperator(firstChar);
        }
        else {
          throw new Exception(Error.unExceptedChar(lineNumber));
        }
      } // End while.
      // Pop any remaining operators
      // and append them to postfix.
      while (!operatorStack.empty()) {
        char op = operatorStack.pop();
        // Any '(' on the stack is not matched.
        if (op == '(')
          throw new Exception(Error.unmatchingOpeningParenthesis(lineNumber));
        postfix.append(op);
        postfix.append(' ');
      }
      // assert: Stack is empty, return result.
      return postfix.toString();
    }
    catch(Exception ex) {
      throw new Exception(Error.emptyStack(lineNumber));
    }
  }

  /** Method to process operators.
      @param op The operator
      @throws EmptyStackException
   */
  private void processOperator(char op) {
    if (operatorStack.empty() || op == '(') {
      operatorStack.push(op);
    }
    else {
      // Peek the operator stack and
      // let topOp be the top operator.
      char topOp = operatorStack.peek();
      if (precedence(op) > precedence(topOp)) {
        operatorStack.push(op);
      }
      else {
        // Pop all stacked operators with equal
        // or higher precedence than op.
        while (!operatorStack.empty() && precedence(op) < precedence(topOp)) {
          operatorStack.pop();
          if (topOp == '(') {
            // Matching '(' popped - exit loop.
            break;
          }
          postfix.append(topOp);
          postfix.append(' ');
          if (!operatorStack.empty()) {
            // Reset topOp.
            topOp = operatorStack.peek();
          }
        }

        // assert: Operator stack is empty or
        //         current operator precedence >
        //         top of stack operator precedence.
        if (op != ')')
          operatorStack.push(op);
      }
    }
  }

  /** Determine whether a character is an operator.
      @param ch The character to be tested
      @return true if ch is an operator
   */
  private boolean isOperator(char ch) {
    return OPERATORS.indexOf(ch) != -1;
  }

  /** Determine the precedence of an operator.
      @param op The operator
      @return the precedence
   */
  private int precedence(char op) {
    return PRECEDENCE[OPERATORS.indexOf(op)];
  }
}

