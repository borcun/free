/**
 * ReturnStatement Class
 * @author borcun
 *
 */
public class ReturnStatement extends Statement
{
	/**
	 * Construct Return object
	 */
	public ReturnStatement() {
		super();
	}
	
	/**
	 * return method call line
	 * @return -999
	 */
	public static final Integer returnMethodCallLine() {
		return -999;
	}
	
	/**
	 * return main call line
	 * @return -9999
	 */
	public static final Integer returnMainCallLine() {
		return -9999;
	}
	
	/**
	 * check if statement is return statement or not
	 * @param st
	 * @return true/false
	 * @throws Exception
	 */
	public boolean isReturn(Statement st) throws Exception {
		// check if statement is return statement or not
		if(st.getLineContent().indexOf("return ") == 0) {
			String[] token = st.getLineContent().split(" ");
			
			// if token.length doesn't equal 2, throw exception
			if(token.length != 2) {
				throw new Exception(Error.isNotOneParameter(st.getLineNumber()));
			}			
			// if parameter of return was declared before, return true
			else {
				// cast string to integer and check if it is real integer
				// if it is not real integer, throw exception
				try {
					Integer.parseInt(token[1]); // parse string to integer
					return true; // return true
				}
				catch(NumberFormatException nfe) {
					// search variable in variable list
					for(int i=0 ; i < VariableStatement.getVariables().size() ; ++i) {
						// if variable is in variable list, check its type
						if(token[1].equals(VariableStatement.getVariables().get(i).getVarName())) {
							// if type is integer, add statement to statement list and return true
							if(VariableStatement.getVariables().get(i).getVarType().equals("int")) {
								if(VariableStatement.getVariables().get(i).getVarValue().equals(null))
									throw new Exception(Error.uninitializedVariable(st.getLineNumber()));
								// return true
								return true;
							}
							// if return parameter is not integer, throw exception
							else
								throw new Exception(Error.notReturnInteger(st.getLineNumber()));
						}	
					}
					// if string isn't variable or integer, throw exception
					throw new Exception(Error.nonDeclarationVariable(st.getLineNumber()));
				}
			} // end of else
		} // end of if
		// return false
		return false;
	}
	
	/**
	 * take return statement value
	 * @param st
	 * @return value that is returned
	 */
	public String takeReturnValue(Statement st) {
		String[] token = st.getLineContent().split(" ");
		
		// cast token[1] to integer
		// if it is an integer, return it
		try {
			Integer.parseInt(token[1]); // parse string to integer
			return token[1];
		} 
		// if token[1] is not an integer, it is a variable, so return its value
		catch(NumberFormatException nfe) {
			// search variable in variable list
			for(int i=0 ; i < VariableStatement.getVariables().size() ; ++i) {
				// if variable is in variable list, check its type
				if(token[1].equals(VariableStatement.getVariables().get(i).getVarName())) {
					return VariableStatement.getVariables().get(i).getVarValue();
				}
			}
		}
		// return null
		return null;
	}
}
