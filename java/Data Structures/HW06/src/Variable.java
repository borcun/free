/**
 * Variable Class
 * @author borcun
 *
 */
public class Variable 
{
	/** variable type */
	private String varType;
	/** variable name */
	private String varName;
	/** variable value */
	private String varValue;
	
	/**
	 * Construct Variable object
	 */
	public Variable() {
		// default constructor
	}
	
	/**
	 * Construct Variable object
	 * @param type
	 * @param name
	 */
	public Variable(String type, String name) {
		this.setVarType(type); // set variable type
		this.setVarName(name); // set variable name
	}

	/**
	 * set variable type
	 * @param varType
	 */
	public void setVarType(String varType) {
		this.varType = varType;
	}

	/**
	 * get variable type
	 * @return varType
	 */
	public String getVarType() {
		return varType;
	}

	/**
	 * set variable name
	 * @param varName
	 */
	public void setVarName(String varName) {
		this.varName = varName;
	}

	/**
	 * get variable name
	 * @return varName
	 */
	public String getVarName() {
		return varName;
	}

	/**
	 * set variable value
	 * @param varValue
	 */
	public void setVarValue(String varValue) {
		this.varValue = varValue;
	}

	/**
	 * get variable value
	 * @return varValue
	 */
	public String getVarValue() {
		return varValue;
	}
}
