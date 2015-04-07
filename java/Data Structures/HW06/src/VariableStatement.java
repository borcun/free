import java.util.ArrayList;

/**
 * DeclarationStatement Class
 * @author borcun
 *
 */
public class VariableStatement extends Statement
{
	/** array list that store variables */
	private static ArrayList<Variable> variables;
	
	/**
	 * Construct VariableStatement object
	 */
	public VariableStatement() {
		super();
		variables = new ArrayList<Variable>();
	}
	
	/**
	 * get array list of variables
	 * @return variables list
	 */
	public static ArrayList<Variable> getVariables() {
		return variables;
	}

	/**
	 * check if statement is declaration statement or not
	 * @param st
	 * @return true/false
	 * @throws Exception 
	 */
	public boolean isDeclaration(Statement st) throws Exception {
		String[] operators = {"+","-","/","*","(",")"};
		// check if statement string has int or double words
		if(st.getLineContent().indexOf("int ") == 0 || st.getLineContent().indexOf("double ") == 0) {
			String[] token = st.getLineContent().split(" "); // split string
			
			// if token.length doesn't equal 2, check if it is method declaration or variable declaration
			if(token.length != 2) {
				// if it is method declaration, return false
				if(token[2].equals("double") || token[2].equals("int"))
					return false;
				// if it is not method declaration, throw exception	
				else
					for(int i=0 ; i < token.length ; ++i ) {
						if(token[i].equals(""))
							throw new Exception(Error.oneSpaceError(st.getLineNumber()));
					}
					throw new Exception(Error.moreDeclaration(st.getLineNumber()));			
			}
			// if token[1] equals int or double, throw exception
			else if(token[1].equals("int") || token[1].equals("double")) {
				throw new Exception(Error.isNotVariableName(st.getLineNumber()));
			}
			// if variable declaration is main declaration, throw exception
			else if(token[0].equals("int") && token[1].equals("main") && MainMethod.getIsDeclared()) {
				throw new Exception(Error.moreMainDeclaration(st.getLineNumber()));
			}
			// add statement to GITLanguage array list and variables array list
			else {
				// compare variable name and operators
				// if variable name contains one of operators, throw exception
				for(int j=0 ; j < operators.length ; ++j) {
					if(token[1].indexOf(operators[j]) != -1)
						throw new Exception(Error.isNotVariableName(st.getLineNumber()));
				}
				
				variables.add(new Variable(token[0],token[1])); // add variable to variables list	
				return true; // return true
			}
		}
		return false;
	}
	
	/**
	 * check if statement is assignment statement or not
	 * @param st
	 * @return true/false
	 * @throws Exception
	 */
	public boolean isAssignment(Statement st) throws Exception {
		int i, temp = 0;
		String[] operators = {"+","-","/","*","(",")"}; // operators
		String[] token = st.getLineContent().split(" "); // split string
		
		// if token.length doesn't equal 3, this statement is not assignment statement, return false
		if(token.length != 3) {
			return false;
		}
		// if middle string doesn't equal =, this statement is not assignment statement, return false
		else if(!token[1].equals("=")) {
			return false;
		}
		// if token.length equals 3 and second string equals =, it may be assignment statement such as length = 4
		else {
			// compare right side of = operator and operators
			// if there is an operator, it is an arithmetic process
			for(int j=0 ; j < operators.length ; ++j) {
				if(token[2].indexOf(operators[j]) != -1)
					return false;
			}
			
			// check if it is an assignment statement or not
			// by searching first string inside variables names
			for(i=0 ; i < variables.size() ; ++i) {
				if(token[0].equals(variables.get(i).getVarName())) {
					for(int j=0 ; j < variables.size() ; ++j) {
						if(i != j && token[2].equals(variables.get(j).getVarName())) {
							if(!variables.get(i).getVarType().equals(variables.get(j).getVarType()))
								throw new Exception(Error.castingError(st.getLineNumber()));
								
							variables.get(i).setVarValue(variables.get(j).getVarValue()); // set variable value
							return true;
						}
					}
					
					// parse string to integer value
					try {
						// parse token2 to temporary integer value
						temp = Integer.parseInt(token[2]);
						variables.get(i).setVarValue(Integer.toString(temp)); // set variable value
						return true; // return true
					} catch(NumberFormatException nfe) {
						throw new Exception(Error.nonDeclarationVariable(st.getLineNumber()));
					}
				} // end of if
			} // end of for
		} // end of else		
		throw new Exception(Error.nonDeclarationVariable(st.getLineNumber()));
	}
}
