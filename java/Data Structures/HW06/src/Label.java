import java.util.ArrayList;
import java.util.Stack;

/**
 * Label Class
 * @author borcun
 *
 */
public class Label
{
	/** label scope that store label-if line numbers */
	private static Stack<Integer> labelStack;
	private static ArrayList<String> labelName;
	
	/**
	 * Construct Label object
	 */
	public Label() {
		labelStack = new  Stack<Integer>();
		labelName = new ArrayList<String>();
	}

	/**
	 * get label scope stack
	 * @return labelStack
	 */
	public static  Stack<Integer> getLabelScope() {
		return labelStack;
	}

	/**
	 * get label name list
	 * @return labelName
	 */
	public static ArrayList<String> getLabelName() {
		return labelName;
	}

	/**
	 * check if statement is label statement or not
	 * @param st
	 * @return true/false
	 * @throws Exception
	 */
	public boolean isLabel(Statement st) throws Exception {
		// if statement starts with "label ", it may be label statement
		if(st.getLineContent().indexOf("label ") == 0) {
			// split according to " " string
			String[] token = st.getLineContent().split(" ");
			
			// if token.lenght < 2, there is just "label ", so throw exception
			if(token.length < 2) {
				throw new Exception(Error.oneParameterLabel(st.getLineNumber()));
			}

			// if token.lenght > 2, there are one more parameter, so throw exception
			else if(token.length > 2) {
				throw new Exception(Error.moreParameterLabel(st.getLineNumber()));
			}
			// if token.lenght == 2, it may be label statement
			// check label parameter if it is an available or not
			else {
				// if label parameter name is label, throw exception
				if(token[1].equals("label"))
					throw new Exception(Error.labelNameError(st.getLineNumber()));
					
				labelStack.push(st.getLineNumber()); // set label start line
				labelName.add(token[1]); // add label name
				return true; // return true
			}
		}
		// return false
		return false;
	}
	
	/**
	 * check if statement is if-condition statement or not
	 * @param st
	 * @return if it is, return line that will be gone, else, -1
	 * @throws Exception
	 */
	public int isIfCondition(Statement st) throws Exception {
		// if statement starts with "if "
		if(st.getLineContent().indexOf("if ") == 0) {
			String[] token = st.getLineContent().split(" ");
			// if token.length < 3, throw exception
			if(token.length < 3) {
				throw new Exception(Error.lessIfParameter(st.getLineNumber()));
			}
			// if token.length > 3, throw exception
			else if(token.length > 3) {
				throw new Exception(Error.moreIfParameter(st.getLineNumber()));
			}
			// if token.length == 3, it may be if-condition statement
			else {
				// search first if-condition parameter in variable list
				for(int i=0 ; i < VariableStatement.getVariables().size() ; ++i) {
					// if there is variable in variable list, check its value
					if(token[1].equals(VariableStatement.getVariables().get(i).getVarName())) {
						// if its value is null, throw exception
						if(VariableStatement.getVariables().get(i).getVarValue().equals(null))
							throw new Exception(Error.uninitializedVariable(st.getLineNumber()));
						// check if label name is in label name list
						try {
							if(labelName.contains(token[2])) {
								// check if variable is bigger than 0 or not 
								if(Integer.parseInt(VariableStatement.getVariables().get(i).getVarValue()) >= 0) {
									// if labelStack is not empty, peek element on top of stack
									if(!labelStack.empty())
										return labelStack.peek();
									
									// throw exception
									throw new Exception(Error.unmatchingLabelIf(st.getLineNumber()));
								}
								// if variable is 0
								else if(Integer.parseInt(VariableStatement.getVariables().get(i).getVarValue()) < 0) {
									// when finish label-if loop, remove label name from label name list
									// and pop label scope from labelStack
									labelName.remove(token[2]);
									labelStack.pop();
									return st.getLineNumber() + 1; // return next line
								}
							}
							// throw exception
							throw new Exception(Error.invalidLabelName(st.getLineNumber()));
						}
						// if there is number format exception, throw exception
						catch(NumberFormatException nfe) {
							throw new Exception(Error.uninitializedVariable(st.getLineNumber()));
						}
					} // end of if
				} // end of for
				// throw exception
				throw new Exception(Error.nonDeclarationVariable(st.getLineNumber()));
			} // end of else
		} // end of if
		return -1; // return -1
	}
}
