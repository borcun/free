import java.util.ArrayList;

/**
 * MainMethod Class
 * @author borcun
 *
 */
public class MainMethod extends Method
{
	/** boolean that store situation of main is declared */
	private static boolean isDeclared;
	/** array list that store main method lines */
	private static ArrayList<Statement> mainMethodLine;
	
	/**
	 * Construct MainScope object
	 */
	public MainMethod() {		
		isDeclared = false; // set isDeclared as false
		mainMethodLine = new ArrayList<Statement>();
	}
	
	/**
	 * set main is declared situation
	 * @param declared
	 */
	public static void setIsDeclared(boolean declared) {
		isDeclared = declared;
	}
	
	/**
	 * get main is declared situation
	 * @return
	 */
	public static boolean getIsDeclared() {
		return isDeclared;
	}
	
	/**
	 * set main method lines
	 * @param mainMethodLine
	 */
	public static void setMainMethodLine(ArrayList<Statement> mainMethodLine) {
		MainMethod.mainMethodLine = mainMethodLine;
	}

	/**
	 * get main method lines
	 * @return
	 */
	public static ArrayList<Statement> getMainMethodLine() {
		return mainMethodLine;
	}

	/**
	 * check if statement is main declaration or not
	 * @param st
	 * @return true/false
	 * @throws Exception
	 */
	public boolean isMain(Statement st) throws Exception {
		// if statement contains of "int main"
		if(st.getLineContent().indexOf("int main") == 0) {
			String[] token = st.getLineContent().split(" "); // split string
			
			// check if any main was declared before
			if(!MainMethod.isDeclared) {
				// if token.length equals 2 and string in second index equals main				
				if(token.length == 2 && token[0].equals("int") && token[1].equals("main")) {
					this.setStartLine(st.getLineNumber()); // set start line
					this.setCallLine(st.getLineNumber()); // set call line like start line
					this.setReturnType(token[0]); // set return type
					this.setMethodName(token[1]); // set method name
					MainMethod.setIsDeclared(true); // set isDeclared as true
					return true; // return true
					
				}
				// if token.length is bigger than 2, check its integer parameter(s)
				if(token.length > 2 && token.length % 2 == 0 && token[0].equals("int") && token[1].equals("main")) {
					for(int i=2 ; i < token.length ; ++i) {
						if(i % 2 == 0 && !token[i].equals("int"))
							throw new Exception(Error.isNotType(st.getLineNumber()));
						else if(i % 2 == 1 && token[i].equals("main"))
							throw new Exception(Error.variableNameMain(st.getLineNumber()));
						// add variable to variable list
						else if(i % 2 == 1 && i > 1) 
							VariableStatement.getVariables().add(new Variable(token[i-1],token[i]));
					}
					this.setStartLine(st.getLineNumber()); // set start line
					this.setCallLine(st.getLineNumber()); // set call line like start line
					this.setReturnType(token[0]); // set return type
					this.setMethodName(token[1]); // set method name
					MainMethod.setIsDeclared(true); // set isDeclared as true
					return true; // return true
				}
			}
			// throw more main exception
			throw new Exception(Error.moreMainDeclaration(st.getLineNumber()));
		}
		return false;
	}
}
