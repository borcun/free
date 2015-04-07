/**
 * CommentStatement Class
 * @author borcun
 *
 */
public class CommentStatement extends Statement
{
	/**
	 * Construct Comment object
	 */
	public CommentStatement() {
		super();
	}
	
	/**
	 * check if statement is comment statement or not
	 * @param st
	 * @return true/false
	 */
	public boolean isComment(Statement st) {
		if(st.getLineContent().startsWith("//")) {
			return true;
		}
		return false;
	}
}
