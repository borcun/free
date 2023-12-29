### Optimization Algorithms<br>
This project is an example implementation project that includes some optimization algorithm implementations. It can be used to demonstrate how the algorithms work. There are seven different optimization methods:
 - Simplex Method Algorithm
   - it reads own input file, then executes simplex algorithm
 - Branch & Bound Algorithm
   - it reads own input file, then executes branch & bound algorithm 
 - Newton-Raphson Algorithm
   - it reads own input file, then executes newton-raphson algorithm
 - Discrete Programming Algorithm
   - it reads own input file, then executes direct search algorithm
 - Direct Search Method
   - it reads own input file, then interacts user to get another input parameters, finally executes direct search (dichotomous apporach) algorithm 
 - Gradient Method
   - it reads own input file, then interacts user to get another input parameters, finally executes gradient algorithm 
 - Linear Combinations Method
   - it reads own input file, then interacts user to get another input parameters, finally executes linear combination algorithm 
   
It shows a menu for the user on command line, so you don't need any graphical interface dependency. However, the algorithms use __sympy__, __scipy__, __numpy__, __csv__ modules for file operations, equation and some matrix/vector operations. All algorithms take own input parameters via csv file, which include coefficients of equaitons defining the problem. These coefficients belong to objective and constraint functions.
