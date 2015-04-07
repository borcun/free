import java.util.ArrayList;

/**
 * OtherMethod Class
 * @author borcun
 *
 */
public class OtherMethod extends Method
{
	/** array list that store method lines */
	private static ArrayList<Statement> methodLine;
	
	/**
	 * Construct OtherMethod object
	 */
	public OtherMethod() {
		// allocate methodLine list
		methodLine = new ArrayList<Statement>();
	}

	/**
	 * get method line
	 * @return
	 */
	public static ArrayList<Statement> getMethodLine() {
		return methodLine;
	}

	/**
	 * check if statement is method declaration or not
	 * @param st
	 * @return true/false
	 * @throws Exception 
	 */
	public boolean isMethodDeclaration(Statement st) throws Exception {
		if(st.getLineContent().indexOf("int ") == 0) {
			String[] token = st.getLineContent().split(" ");

			// if token.length < 3, return false
			// statement isn't a method declaration
			if(token.length < 3 )
				return false;
			// if token.length > 3, check if it is method declaration or not
			// if token.length % 2 == 1, method declaration has one more or less word
			else if(token.length % 2 == 1)
				throw new Exception(Error.moreDeclaration(st.getLineNumber()));
			// parse method declaration string by string and control string's content
			else {
				for(int i=0 ; i < token.length ; ++i) {
					// if i%2 == 0, word have not to be int keyword, so throw exception
					if(i % 2 == 0 && !token[i].equals("int"))
						throw new Exception(Error.faultParameter(st.getLineNumber()));
					// if i%2 == 1, word have to be int keyword, so throw exception
					else if(i % 2 == 1 && token[i].equals("int"))
						throw new Exception(Error.faultParameter(st.getLineNumber()));
					// if word or keyword is true, add variable to variable list as method name + variable name
					else if(i % 2 == 1 && !token[i].equals("int") && i>1 ) {
						this.setParameterType(token[i-1]); // set parameter type
						this.setParameterName(token[i]); // set parameter name
						VariableStatement.getVariables().add(new Variable(token[i-1],token[i]));
					}
				}
				// add method and its attributes(declaration line,type,name) to method list
				this.setStartLine(st.getLineNumber());
				this.setReturnType(token[0]);
				this.setMethodName(token[1]);
				return true; // return true
			}
		} // end of if			
		return false;
	}
	
	/**
	 * check if statement is calling method or not
	 * @param st
	 * @return true/false
	 * @throws Exception
	 */
	public boolean isCallMethod(Statement st) throws Exception {
		// check if statement start with "call " string
		if(st.getLineContent().indexOf("call ") == 0 && 
			st.getLineContent().indexOf("call print ") != 0 && st.getLineContent().indexOf("call scan ") != 0 ) {
			
			String[] token = st.getLineContent().split(" "); // split string
						
			// if token.length < 4 and statement starts with "call ", this statement is false
			if(token.length < 4)
				throw new Exception(Error.lessParameter(st.getLineNumber()));
			// if token.length > 4 and statement starts with "call ", this statement is false
			else if(token.length > 4)
				throw new Exception(Error.moreParameter(st.getLineNumber()));
			// if statement is true, search its name in method list, next parse its parameters
			else {
				// first parameter may be integer, therefore
				// try to parse first parameter string to integer value
				// set method first parameter value as token[2]
				try {
					// parse token[2] to integer value
					Integer.parseInt(token[2]);
					// look up second parameter
					for(int i=0 ; i < VariableStatement.getVariables().size() ; ++i) {
						// if second parameter is in variable name list, return line number of method
						if(token[3].equals(VariableStatement.getVariables().get(i).getVarName())) {
							this.setCallLine(st.getLineNumber()); // set call line
							// search second parameter(return parameter) in variable list
							for(int j=0 ; j < VariableStatement.getVariables().size() ; ++j) {
								if(this.getParameterName().equals(VariableStatement.getVariables().get(j).getVarName())) {
									VariableStatement.getVariables().get(j).setVarValue(token[2]);
									break;
								}
							}
							
							return true; // return true
						}
					}
					// throw exception
					throw new Exception(Error.nonDeclarationVariable(st.getLineNumber()));
				}
				// if there is any exception, string is not numeric
				// so, check if string is declared variable name or not
				catch(NumberFormatException nfe) {
					// search string in variable list
					for(int i=0 ; i < VariableStatement.getVariables().size() ; ++i) {
						// if string is in variable name list, check second parameter
						if(token[2].equals(VariableStatement.getVariables().get(i).getVarName())) {
							// if second parameter is numeric, throw exception
							try {
								// try to parse token[3] to integer value
								Integer.parseInt(token[3]);
								throw new Exception(Error.numericSecondParameterForMethod(st.getLineNumber()));
							}
							// if second parameter is not numeric, check if it is in variable list
							catch(NumberFormatException nfe2) {
								for(int j=0 ; j < VariableStatement.getVariables().size() ; ++j) {
									// if second parameter is in variable name list, return line number of method
									if(token[3].equals(VariableStatement.getVariables().get(j).getVarName())) {
										this.setCallLine(st.getLineNumber()); // set call line
										// search second parameter(return parameter) in variable list
										for(int k=0 ; k < VariableStatement.getVariables().size() ; ++k) {
											if(this.getParameterName().equals(VariableStatement.getVariables().get(k).getVarName())) {
												VariableStatement.getVariables().get(k).setVarValue(VariableStatement.getVariables().get(i).getVarValue());
												break;
											}
										}
										return true; // return start line
									}
								}
							}
							// if exception is numericSecondParameterForMethod, throw it
							catch(Error error) {
								throw new Exception(Error.numericSecondParameterForMethod(st.getLineNumber()));
							}
						} // end of if
					} // end of for
					// throw exception
					throw new Exception(Error.nonDeclarationVariable(st.getLineNumber()));
				} // end of catch
			} // end of else
		} // end of if
		return false;
	}
	
	/**
	 * check if statement is call print or call scan method or not
	 * @param st
	 * @return true/false
	 * @throws Exception
	 */
	public boolean isCallPrintScan(Statement st) throws Exception {
		// check if statement start with "call " string
		if(st.getLineContent().indexOf("call print ") == 0 || st.getLineContent().indexOf("call scan ") == 0) {
			String[] token = st.getLineContent().split(" "); // split string
			
			// if token.length < 4 and statement starts with "call ", this statement is false
			if(token.length < 4)
				throw new Exception(Error.lessParameter(st.getLineNumber()));
			// if token.length > 4 and statement starts with "call ", this statement is false
			else if(token.length > 4)
				throw new Exception(Error.moreParameter(st.getLineNumber()));
			// if call method is print method
			else if(token[0].equals("call") && token[1].equals("print")) {
				// parse first parameter to integer
				try {
					System.out.println(Integer.parseInt(token[2])); // print on screen
					return true;
				} 
				// if first parameter isn't an integer
				catch(NumberFormatException nfe) {
					// search it in variable list
					for(int i=0 ; i < VariableStatement.getVariables().size() ; ++i) {
						// if it is in variable list, print on screen and return true
						if(token[2].equals(VariableStatement.getVariables().get(i).getVarName())) {
							if(VariableStatement.getVariables().get(i).getVarName().equals(null))
								throw new Exception(Error.uninitializedVariable(st.getLineNumber()));
								
							System.out.println(VariableStatement.getVariables().get(i).getVarValue());
							return true;
						}
					}					
					// if it isn't in variable list, throw exception
					throw new Exception(Error.nonDeclarationMethod(st.getLineNumber()));
				}
			}
			// if call method is scan method
			else if(token[0].equals("call") && token[1].equals("scan")) {
				// parse second parameter to integer
				try {
					Integer.parseInt(token[2]);
					for(int i=0 ; i < VariableStatement.getVariables().size() ; ++i) {
						if(token[3].equals(VariableStatement.getVariables().get(i).getVarName())) {
							VariableStatement.getVariables().get(i).setVarValue(token[3]); // set variable value
							return true; // return true
						}
					}
				} 
				// if first parameter isn't integer, search it in variable list
				catch(NumberFormatException nfe) {
					for(int i=0 ; i < VariableStatement.getVariables().size() ; ++i) {
						// if first parameter is in variable list, check second parameter
						if(token[2].equals(VariableStatement.getVariables().get(i).getVarName())) {
							// parse second parameter to integer
							try {
								Integer.parseInt(token[3]);
								// if second parameter is an integer, throw an exception
								throw new Exception(Error.numericSecondParameterForMethod(st.getLineNumber()));
							}
							// if second parameter isn't an integer, search it in variable list
							catch(NumberFormatException nfe1) {
								for(int j=0 ; j < VariableStatement.getVariables().size() ; ++j) {
									// if it is in variable list
									if(token[3].equals(VariableStatement.getVariables().get(j).getVarName())) {
										// set second parameter value as first parameter value
										VariableStatement.getVariables().get(j).setVarValue(VariableStatement.getVariables().get(i).getVarValue());
										return true; // return true
									}
								}
							}
						} // end of if
					} // end of for
					throw new Exception(Error.nonDeclarationMethod(st.getLineNumber()));
				} // end of catch
			} // end of else-if
		}
		return false;
	}
}
