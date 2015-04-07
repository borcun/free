import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.ArrayList;
/**
 * Main Class
 * @author borcun
 *
 */
public class Main 
{
	/** arrayList store statement of source code file */
	private static ArrayList<Statement> sourceCode = new ArrayList<Statement>();
	private static MainMethod mainMethod = new MainMethod();
	private static OtherMethod otherMethod = new OtherMethod();
	private static VariableStatement vs = new VariableStatement();
	private static ArithmeticStatement as = new ArithmeticStatement();
	private static CommentStatement cs = new CommentStatement();
	private static ReturnStatement rs = new ReturnStatement();
	private static Label label = new Label();
	private static int i = 0;
	private static int j = 0;
	private static int temp;
	
	/** Main method */
	public static void main(String[] args) {
		RandomAccessFile raf = null; // random access file
		String line; // string that stores a line of source code file
		// before execute source code, add source code line to an array list
		try {
			// open file with argument args[0]
			raf = new RandomAccessFile(args[0],"r");
			
			// read file alternately until end of file
			while((line = raf.readLine()) != null) {
				line = line.trim(); // trim spaces of line
				sourceCode.add(new Statement(line,i+1)); // add line to source code list
				++i; // increase index
			} // end of while
			raf.close(); // close file
		}
		// catch exception if there isn't file
		catch (FileNotFoundException e) {
			System.out.println(e.getMessage()); // print error message
			System.exit(0); // exit from system
		}
		// catch exception if there is IO exception
		catch (IOException e) {
			System.out.println(e.getMessage()); // print error message
			System.exit(0); // exit from system
		} 
		// catch exception if there is any exception
		catch (Exception e) {
			System.out.println(e.getMessage()); // print error message
			System.exit(0); // exit from system
		}
		
		i=0; // set i as 0
		
		// take main and other methods scopes(start line, call line, finish line, method name and return type)
		while(i < sourceCode.size()) {
			try {
				// if statement is main method declaration
				if(mainMethod.isMain(sourceCode.get(i))) {
					while(!sourceCode.get(i).getLineContent().equals("end")) {
						MainMethod.getMainMethodLine().add(sourceCode.get(i)); // add line to main method line
						++i; // increase index
					}
					MainMethod.getMainMethodLine().add(sourceCode.get(i)); // add end statement to main method line
					mainMethod.setFinishLine(i); // add finish line
					MainMethod.setIsDeclared(false); // set main declared situation
				}
				// if statement is method declaration
				else if(otherMethod.isMethodDeclaration(sourceCode.get(i))) {
					while(!sourceCode.get(i).getLineContent().equals("end")) {
						OtherMethod.getMethodLine().add(sourceCode.get(i)); // add line to method line
						++i; // increase index
					}
					OtherMethod.getMethodLine().add(sourceCode.get(i)); // add end statement to method line
					otherMethod.setFinishLine(i); // add finish line
				}					
			} 
			catch (Exception e) {}
			
			++i; // increase index
		} // end of while
		
		i = 0;
		
		// run main source code
		while(runMethod(MainMethod.getMainMethodLine(),i)) {
			// if mainMethod returns true, run method source code
			// run until method source code finishes, next 
			// continue to run main source code from remain it
			while(runMethod(OtherMethod.getMethodLine(),j));
			// set method source code line, jump over call method line
			i = mainMethod.getCallLine()+1;
		}
		
		System.exit(0); // exit from program
	}
	
	/**
	 * run method
	 * @param start
	 * @param finish
	 * @return true / false
	 */
	private static boolean runMethod(ArrayList<Statement> methodList, int index) {
		// until end of method list
		while(index < methodList.size()) {
			try {
				// if statement is main statement
				if(mainMethod.isMain(methodList.get(index))) {
					System.out.println("Main");
					Method.setSCOPE("main");
				}
				// if statement is variable declaration statement
				else if(vs.isDeclaration(methodList.get(index))) {
					System.out.println("Variable Declaration");
				}
				// if statement is variable assignment statement
				else if(vs.isAssignment(methodList.get(index))) {
					System.out.println("Assignment");
				}
				// if statement is arithmetic statement
				else if(as.isArithmetic(methodList.get(index)) != null) {
					as.evaluateResult(new Statement(as.isArithmetic(methodList.get(index)),methodList.get(index).getLineNumber()));
					System.out.println("Arithmetic Statement");
				}
				// if statement is comment statement
				else if(cs.isComment(methodList.get(index))) {
					System.out.println("// Comment");
				}
				// if statement is method declaration
				else if(otherMethod.isMethodDeclaration(methodList.get(index))) {
					System.out.println("Method Declaration");
				}
				// if statement is method call
				else if(otherMethod.isCallMethod(methodList.get(index))) {
					System.out.println("Call Method");
					mainMethod.setCallLine(index);
					Method.setSCOPE("method");
					return true;
				}
				// if statement is print or scan call
				else if(otherMethod.isCallPrintScan(methodList.get(index))) {
					System.out.println("Print or Scan");
				}
				// if statement is label statement
				else if(label.isLabel(methodList.get(index))) { 
					System.out.println("Label Statement");
				}
				// if statement is if condition statement
				else if((temp = label.isIfCondition(methodList.get(index))) != -1) {
					System.out.println("If Condition Statement");
					index = temp - mainMethod.getStartLine();
				}
				// if statement is return statement
				else if(rs.isReturn(methodList.get(index))) {
					System.out.println("Return Statement");
					// check if return statement belongs to main or not
					if(!Method.getSCOPE().equals("main")) {
						String[] token = MainMethod.getMainMethodLine().get(mainMethod.getCallLine()).getLineContent().split(" ");
						// search return value in variable list
						for(int k=0 ; k < VariableStatement.getVariables().size() ; ++k) {
							if(token[3].equals(VariableStatement.getVariables().get(k).getVarName())) {
								VariableStatement.getVariables().get(k).setVarValue(rs.takeReturnValue(methodList.get(index)));
								break;
							}
						}
					}						
				}
				// if statement is end, terminate scope
				else if(Method.isEnd(methodList.get(index)) != -1) {
					System.out.println("End Statement");
				}
				// if statement is empty line
				else if(methodList.get(index).getLineContent().equals("")){
					System.out.println("Empty");
				}
				// if statement is unknown line, throw exception
				else
					throw new Exception(Error.unknownError(methodList.get(index).getLineNumber()));
			}
			// catch exception if there is any GITLanguage exception
			catch (Exception e) {
				System.out.println(e.getMessage());
				System.exit(0); // exit
			}
			++index; // increase index
		}
		return false;
	}
}
