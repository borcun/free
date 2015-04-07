/**
 * Statement Abstract Class
 * @author borcun
 *
 */
public class Statement 
{
	/** content of line */
	private String lineContent;
	/** line number */
	private int lineNumber;
	
	/**
	 * Construct Statement object
	 */
	public Statement() {
		this.setLineContent("");
		this.setLineNumber(-1);
	}
	
	/**
	 * Construct Statement object with content and number of line
	 * @param content
	 * @param number
	 */
	public Statement(String content, int number) {
		this.setLineContent(content);
		this.setLineNumber(number);
	}
	
	/**
	 * set content of line
	 * @param content
	 */
	public void setLineContent(String content) {
		this.lineContent = content;
	}

	/**
	 * get content of line
	 * @return content of line
	 */
	public String getLineContent() {
		return lineContent;
	}

	/**
	 * set line number
	 * @param number
	 */
	public void setLineNumber(int number) {
		this.lineNumber = number;
	}

	/**
	 * get line number
	 * @return line number
	 */
	public int getLineNumber() {
		return lineNumber;
	}	
}
