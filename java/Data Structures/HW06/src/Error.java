/**
 * Error Class
 * @author borcun
 *
 */
public class Error extends Exception
{
	/**	Serial ID */
	private static final long serialVersionUID = 395053645082116774L;
	/** public error string */
	public static String errorString;
	
	/**
	 * more one same variables declaration error
	 * @param i
	 * @return errorString
	 */
	public static String moreDeclaration(int i) {
		errorString = "line " + i + " : ONE VARIABLE CAN BE DECLARED IN A LINE";
		return errorString;
	}
	
	/**
	 * error is using main word for variable name
	 * @param i
	 * @return errorString
	 */
	public static String variableNameMain(int i) {
		errorString = "line " + i + " : VARIABLE NAME CAN NOT BE main";
		return errorString;
	}
	
	/**
	 * error is using non-declaration variable
	 * @param i
	 * @return errorString
	 */
	public static String nonDeclarationVariable(int i) {
		errorString = "line " + i + " : VARIABLE IS NOT DECLARED";
		return errorString;
	}
	
	/**
	 * more one main declaration error
	 * @param i
	 * @return errorString
	 */
	public static String moreMainDeclaration(int i) {
		errorString = "line " + i + " : ONE MORE MAIN CAN NOT BE DECLARED";
		return errorString;
	}
	
	/**
	 * error is using a type except int and double
	 * @param i
	 * @return errorString
	 */
	public static String isNotType(int i) {
		errorString = "line " + i + " : TYPE NAME ERROR";
		return errorString;
	}
	
	/**
	 * error is making type name variable name
	 * @param i
	 * @return errorString
	 */
	public static String isNotVariableName(int i) {
		errorString = "line " + i + " : VARIABLE NAME CAN NOT BE A TYPE NAME";
		return errorString;
	}

	/**
	 * uninitialized variable error
	 * @param i
	 * @return errorString
	 */
	public static String uninitializedVariable(int i) {
		errorString = "line " + i + " : VARIABLE IS NOT INITIALIZED";
		return errorString;
	}
	
	/**
	 * more one parameter for return error
	 * @param i
	 * @return errorString
	 */
	public static String isNotOneParameter(int i) {
		errorString = "line " + i + " : RETURN CAN TAKE ONE PARAMETER";
		return errorString;
	}
	
	/**
	 * no closed scope error
	 * @param i
	 * @return errorString
	 */
	public static String notCloseScope(int i) {
		errorString = "line " + i + " : NOT ENOUGH END STATEMENT";
		return errorString;
	}
	
	/**
	 * l-value error
	 * @param i
	 * @return errorString
	 */
	public static String lValueError(int i) {
		errorString = "line " + i + " : L-VALUE CAN NOT BE NUMBER";
		return errorString;
	}
	
	/**
	 * casting error
	 * @param i
	 * @return errorString
	 */
	public static String castingError(int i) {
		errorString = "line " + i + " ASSIGNMENT TYPES ARE DIFFERENT";
		return errorString;
	}
	
	/**
	 * division by zero error
	 * @param i
	 * @return errorString
	 */
	public static String divisionByZero(int i) {
		errorString = "line " + i + " DIVISION BY ZERO";
		return errorString;
	}
	
	/**
	 * unexcepted character encountered error
	 * @param i
	 * @return errorString
	 */
	public static String unExceptedChar(int i) {
		errorString = "line " + i + " : UNEXCEPTED CHARACTER ENCOUNTERED";
		return errorString;
	}
	
	/**
	 * unmatching opening parenthesis error
	 * @param i
	 * @return errorString
	 */
	public static String unmatchingOpeningParenthesis(int i) {
		errorString = "line " + i + " : UNMATCHED OPENING PARENTHESIS";
		return errorString;
	}
	
	/**
	 * empty stack error
	 * @param i
	 * @return errorString
	 */
	public static String emptyStack(int i) {
		errorString = "line " + i + " : STACK IS EMPTY";
		return errorString;
	}

	/**
	 * return before main error
	 * @param i
	 * @return errorString
	 */
	public static String returnBeforeMain(int i) {
		errorString = "line " + i + " : THERE ISNT MAIN FUNCTION";
		return errorString;
	}
	
	/**
	 * one more return error
	 * @param i
	 * @return errorString
	 */
	public static String moreReturn(int i) {
		errorString = "line " + i + " : ONE MORE RETURN CAN NOT BE WRITTEN";
		return errorString;
	}
	
	/**
	 * not return integer
	 * @param i
	 * @return errorString
	 */
	public static String notReturnInteger(int i) {
		errorString = "line " + i + " : MAIN MUST RETURN INTEGER";
		return errorString;
	}
	
	/**
	 * end before main or return error
	 * @param i
	 * @return errorString
	 */
	public static String endBeforeMainOrReturn(int i) {
		errorString = "line " + i + " : THERE ISNT MAIN or RETURN";
		return errorString;
	}
	
	/**
	 * operation before main error
	 * @param i
	 * @return errorString
	 */
	public static String operationBeforeMain(int i) {
		errorString = "line " + i + " : ANY OPERATION CAN NOT MADE BEFORE MAIN";
		return errorString;
	}
	
	/**
	 * one more end error
	 * @param i
	 * @return errorString
	 */
	public static String moreEnd(int i) {
		errorString = "line " + i + " : ONE MORE END CAN NOT BE WRITTEN";
		return errorString;
	}
	
	/**
	 * operation after return error
	 * @param i
	 * @return errorString
	 */
	public static String operationAfterReturn(int i) {
		errorString = "line " + i + " : ANY OPERATION CAN NOT MADE AFTER RETURN";
		return errorString;
	}
	
	/**
	 * operation after end error
	 * @param i
	 * @return errorString
	 */
	public static String operationAfterEnd(int i) {
		errorString = "line " + i + " : ANY OPERATION CAN NOT MADE AFTER END";
		return errorString;
	}
	
	/**
	 * unmatched scope error
	 * @param i
	 * @return errorString
	 */
	public static String unmatchedScope(int i) {
		errorString = "line " + i + " : UNMATCHED SCOPE ERROR";
		return errorString;
	}
	
	/**
	 * error that is there are one more space character between type and variable name 
	 * @param i
	 * @return errorString
	 */
	public static String oneSpaceError(int i) {
		errorString = "line " + i + " : ONE SPACE CHARACTER MUST BE BETWEEN TYPE AND VARIABLE";
		return errorString;
	}
	
	/**
	 * error that is method declaration is false
	 * @param i
	 * @return errorString
	 */
	public static String faultParameter(int i) {
		errorString = "line " + i + " : METHOD DECLARATION ERROR";
		return errorString;
	}

	/**
	 * error is calling method has less parameter
	 * @param i
	 * @return errorString
	 */
	public static String lessParameter(int i) {
		errorString = "line " + i + " : LESS PARAMETER FOR METHOD CALL";
		return errorString;
	}
	
	/**
	 * error is calling method has more parameter
	 * @param i
	 * @return errorString
	 */
	public static String moreParameter(int i) {
		errorString = "line " + i + " : MORE PARAMETER FOR METHOD CALL";
		return errorString;
	}
	
	/**
	 * non-declaration method error
	 * @param i
	 * @return errorString
	 */
	public static String nonDeclarationMethod(int i) {
		errorString = "line " + i + " : METHOD IS NOT DECLARED";
		return errorString;
	}
	
	/**
	 * numeric second parameter error for call method
	 * @param i
	 * @return errorString
	 */
	public static String numericSecondParameterForMethod(int i) {
		errorString = "line " + i + " : SECOND PARAMATER CAN NOT BE INTEGER";
		return errorString;
	}
	
	/**
	 * error that is label takes one parameter or less
	 * @param i
	 * @return errorString
	 */
	public static String oneParameterLabel(int i) {
		errorString = "line " + i + " : LABEL HAVE TO TAKE ONE PARAMETER";
		return errorString;
	}
	
	/**
	 *  error that is label takes one parameter or more
	 * @param i
	 * @return errorString
	 */
	public static String moreParameterLabel(int i) {
		errorString = "line " + i + " : LABEL CAN TAKE ONE PARAMETER";
		return errorString;
	}
	
	/**
	 *  error that is label isn't available
	 * @param i
	 * @return errorString
	 */
	public static String labelNameError(int i) {
		errorString = "line " + i + " : LABEL NAME CAN NOT BE LABEL";
		return errorString;
	}
	
	/**
	 * error that is there wasn't label name
	 * @param i
	 * @return errorString
	 */
	public static String invalidLabelName(int i) {
		errorString = "line " + i + " : INVALID LABEL NAME";
		return errorString;
	}
	
	/**
	 *  error that is if-statement parameters are less than 3
	 * @param i
	 * @return errorString
	 */
	public static String lessIfParameter(int i) {
		errorString = "line " + i + " : LESS IF PARAMETER";
		return errorString;
	}
	
	/**
	 * error that is if-statement parameters are more than 3
	 * @param i
	 * @return errorString
	 */
	public static String moreIfParameter(int i) {
		errorString = "line " + i + " : MORE IF PARAMETER";
		return errorString;
	}
	
	/**
	 * unmatching label-if error
	 * @param i
	 * @return errorString
	 */
	public static String unmatchingLabelIf(int i) {
		errorString = "line " + i + " : UNMATCHING LABEL-IF";
		return errorString;		
	}
	
	/**
	 * error for unknown errors
	 * @param i
	 * @return errorString
	 */
	public static String unknownError(int i) {
		errorString = "line " + i + " : INVALID STATEMENT ERROR";
		return errorString;
	}
}
