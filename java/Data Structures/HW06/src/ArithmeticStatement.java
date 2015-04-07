/**
 * ArithmeticStatement Class
 * @author borcun
 *
 */
public class ArithmeticStatement extends Statement
{
	/** string array stores operators */
	private final String[] operators = {"+","-","/","*","(",")"};
		
	/**
	 * Construct Arithmetic object
	 */
	public ArithmeticStatement() {
		super();
	}

	/**
	 * check if statement is arithmetic statement or not
	 * @return true/false
	 * @throws Exception 
	 */
	public String isArithmetic(Statement st) throws Exception {
		int checkOp = 0;
		String token1[] = st.getLineContent().split(" = ");
		// if token1.length equals 2, it may be arithmetic operation such as i = ( 4 * 6 ) / 7
		if(token1.length == 2) {
			// check if l-value is declared variable or not
			for(int i=0 ; i < VariableStatement.getVariables().size() ; ++i) {
				if(token1[0].equals(VariableStatement.getVariables().get(i).getVarName())) {
					// check if right side of = operator has any operator or not
					// if right side of = operator has any operator, it may be true operation
					for(int j=0 ; j < operators.length ; ++j) {
						if(token1[1].indexOf(operators[j]) != -1) {
							++checkOp;
							break;
						}
					}
					
					// if checkOp equals 0, there isn't any operator
					if(checkOp == 0)
						return null;
					
					String token2[] = st.getLineContent().split(" "); // split by space
					
					// start from first element of right side of = operator
					// check them alternately if they are integer/double, operand or declared variable name
					// after (first element is declared variable name) and (second element is = operator) are true
					for(int j=2 ; j < token2.length ; ++j) {
						// parse token2[j] to integer to check if it is integer/double, operand or declared variable name
						try {
							Integer.parseInt(token2[j]); // parse string to integer
						}
						// if exception is NumberFormatException, it may be operand or declared variable name
						catch(NumberFormatException nfe) {
							// check if element is operator or not
							if(!isOperator(token2[j])) {						
								// if element isn't operator, check if it is declared variable name or not
								for(int k=0 ; k < VariableStatement.getVariables().size() ; ++k) {
									if(token2[j].equals(VariableStatement.getVariables().get(k).getVarName())) {
										// if declared variable value is null, it is not initialized, throw exception
										if(VariableStatement.getVariables().get(k).getVarValue() == null)
											throw new Exception(Error.uninitializedVariable(st.getLineNumber()));
											
										// if variable value is initialized, set variable name as variable value
										token2[j] = VariableStatement.getVariables().get(k).getVarValue();
										break; // if it is declared variable name, break loop
									}
									
									// if k equals size of variables, that means to variable list doesn't contain token2[j]
									// in this case, token2[j] isn't integer/double, operator or declared variable name
									// so, throw nonDeclarationVariableError exception
									if(k == VariableStatement.getVariables().size())
										throw new Exception(Error.nonDeclarationVariable(st.getLineNumber()));
								} // end of for
							} // end of if
						} // end of catch
					} // end of for
					
					String temp = ""; // create temporary string
					// assign statement string with new statement string that is changed its variable name as variable value
					for(int j=0 ; j < token2.length ; ++j) {
						temp += token2[j]; // add token2[j]
						temp += " "; // add " "
					}
		
					// if arithmetic statement is true, return true
					return temp;					
				} // end of if
			} // end of for
			
			// parse token1[0] to integer to check if right side is integer or string
			try {
				Integer.parseInt(token1[0]); // parse string to integer
			}
			// if exception is NumberFormatException, throw nonDeclarationVariable exception
			catch(NumberFormatException nfe) {
				// if l-value wasn't declared, throw exception
				throw new Exception(Error.nonDeclarationVariable(st.getLineNumber()));
			}
			// if exception is l-value exception, throw lValueError
			throw new Exception(Error.lValueError(st.getLineNumber()));
		} // end of if
		
		return null;
	}
	
	/**
	 * check if parameter string is any operator or not
	 * @param str
	 * @return true/false
	 */
	private boolean isOperator(String str) {
		final String operatorsStr = "+-/*()"; // operators string
		
		// check if operatorsStr contains str or not
		if(operatorsStr.indexOf(str) != -1)
			return true; // return true
		
		return false; // return false
	}
	
	/**
	 * evaluate result of arithmetic statement
	 * @param st
	 * @throws Exception
	 */
	public void evaluateResult(Statement st) throws Exception {
		int tempInt;
		PostfixEvaluator pe = new PostfixEvaluator();
		InfixToPostfixParens itpp = new InfixToPostfixParens();
		String token[] = st.getLineContent().split(" = ");
		
		// send right side of = operator and line number to convert method
		// send statement that is convert from infix to postfix to eval method
		// assign result to tempInt variable
		tempInt = pe.eval(itpp.convert(token[1],st.getLineNumber()),st.getLineNumber());
		
		// search variable in variable list
		for(int i=0 ; i < VariableStatement.getVariables().size() ; ++i) {
			// when variable is found, set its variable value
			if(token[0].equals(VariableStatement.getVariables().get(i).getVarName())) {
				VariableStatement.getVariables().get(i).setVarValue(String.valueOf(tempInt));
				break; // break loop
			}
		}
		// finish method
		return;
	}
}
