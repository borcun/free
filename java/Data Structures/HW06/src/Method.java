/**
 * Method Class
 * @author borcun
 *
 */
public class Method 
{
	/** SCOPE string of method */
	private static String SCOPE;
	/** start line number of method */
	private int startLine;
	/** call line number of method */
	private int callLine;
	/** finish line number of method */
	private int finishLine;
	/** return type of method */
	private String returnType;
	/** method name */
	private String methodName;
	/** return value of method */
	private String returnVaule;
	/** parameter type of method */
	private String parameterType;
	/** parameter name of method */
	private String parameterName;
	
	/**
	 * Construct Method object
	 */
	public Method() {
		this.setStartLine(-1);
		this.setCallLine(-1);
		this.setFinishLine(-1);
		this.setReturnType(null);
		this.setMethodName(null);
		this.setReturnVaule(null);
	}
	
	/**
	 * Construct Method object
	 * @param start
	 * @param call
	 * @param name
	 * @param pType
	 * @param pName
	 */
	public Method(int start,String type,String name, String pType, String pName) {
		this.setStartLine(start);
		this.setReturnType(type);
		this.setMethodName(name);
		this.setParameterType(pType);
		this.setParameterName(pName);
	}

	/**
	 * set method scope name
	 * @param scope
	 */
	public static void setSCOPE(String scope) {
		SCOPE = scope;
	}

	/**
	 * get method scope name
	 * @return
	 */
	public static String getSCOPE() {
		return SCOPE;
	}

	/**
	 * set method scope
	 * @param startLine
	 */
	public void setStartLine(int startLine) {
		this.startLine = startLine;
	}

	/**
	 * get method start line
	 * @return startLine
	 */
	public int getStartLine() {
		return startLine;
	}
	
	/**
	 * set method call line
	 * @param callLine
	 */
	public void setCallLine(int callLine) {
		this.callLine = callLine;
	}

	/**
	 * get method call line
	 * @return callLine
	 */
	public int getCallLine() {
		return callLine;
	}

	/**
	 * set method finish line
	 * @param finishLine
	 */
	public void setFinishLine(int finishLine) {
		this.finishLine = finishLine;
	}

	/**
	 * get method finish line
	 * @return finishLine
	 */
	public int getFinishLine() {
		return finishLine;
	}

	/**
	 * set method return type
	 * @param returnType
	 */
	public void setReturnType(String returnType) {
		this.returnType = returnType;
	}

	/**
	 * get method return type
	 * @return returnType
	 */
	public String getReturnType() {
		return returnType;
	}

	/**
	 * set method name
	 * @param methodName
	 */
	public void setMethodName(String methodName) {
		this.methodName = methodName;
	}

	/**
	 * get method name
	 * @return methodName
	 */
	public String getMethodName() {
		return methodName;
	}

	/**
	 * set method return value
	 * @param returnVaule
	 */
	public void setReturnVaule(String returnVaule) {
		this.returnVaule = returnVaule;
	}

	/**
	 * get method return value
	 * @return returnValue
	 */
	public String getReturnVaule() {
		return returnVaule;
	}
	
	/**
	 * set method parameter type
	 * @param parameterType
	 */
	public void setParameterType(String parameterType) {
		this.parameterType = parameterType;
	}

	/**
	 * get method parameter type
	 * @return parameterType
	 */
	public String getParameterType() {
		return parameterType;
	}

	/**
	 * set method parameter name
	 * @param parameterName
	 */
	public void setParameterName(String parameterName) {
		this.parameterName = parameterName;
	}

	/**
	 * get method parameter name
	 * @return parameterName
	 */
	public String getParameterName() {
		return parameterName;
	}

	/**
	 * check if statement is end or not
	 * @param st
	 * @return -1 : isn't end, 0 : is main method, 1 : is other method 
	 */
	public static int isEnd(Statement st) {
		if(st.getLineContent().equals("end")) {
			// if scope belongs to main, return 0
			// o indicates end of program
			if(SCOPE.equals("main"))
				return 0;
			// if scope isn't main scope, return 1
			// 1 indicates going on program, set SCOPE as "main"
			SCOPE = "main";
			return 1;
		}
		// if statement isn't end, return -1
		return -1;
	}
}
