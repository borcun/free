import numpy as np
import pandas as pd
from scipy.optimize import linprog

iteration_count = 0  # Counter for iteration number

def branch_and_bound(c, A_ub, b_ub, bounds, integer_vars, incumbent=None):
    global iteration_count
    iteration_count += 1
    
    # Solve the linear programming relaxation
    result = linprog(c, A_ub=A_ub, b_ub=b_ub, bounds=bounds, method='highs', options={'presolve': False})

    # Check if the relaxation is infeasible
    if result is None or result.status == 2:
        print(f"{'*' * 30}\nIteration {iteration_count}: Infeasible relaxation.")
        return incumbent  # Prune the branch

    # Check if the relaxation's objective value is worse than the current incumbent
    if incumbent is not None and -result.fun <= incumbent[0]:
        print(f"{'*' * 30}\nIteration {iteration_count}: Pruning branch (Objective value not improving).")
        return incumbent  # Prune the branch

    # Check if the solution is integer
    if all(np.floor(result.x[integer_vars]) == result.x[integer_vars]):
        # Update the incumbent if a better solution is found
        if incumbent is None or -result.fun > incumbent[0]:
            incumbent = [-result.fun, result.x]
            print(f"{'*' * 30}\nIteration {iteration_count}: Improved incumbent found. x={result.x}, f={-result.fun}")
        else:
            print(f"{'*' * 30}\nIteration {iteration_count}: Integer solution found but not improving incumbent.")
    else:
        print(f"{'*' * 30}\nIteration {iteration_count}: Non-integer solution found. Exploring branches. x={result.x}, f={-result.fun}")

    # Identify the index of the variable with a non-integer value
    non_integer_var_index = np.argmax(np.floor(result.x[integer_vars]) != result.x[integer_vars])
    
    # Create two subproblems by adding new bounds for the identified variable
    bounds_left = bounds.copy()
    bounds_left[non_integer_var_index] = (bounds_left[non_integer_var_index][0], np.floor(result.x[integer_vars][non_integer_var_index]))
    subproblem_left = branch_and_bound(c, A_ub, b_ub, bounds_left, integer_vars, incumbent)

    bounds_right = bounds.copy()
    bounds_right[non_integer_var_index] = (np.ceil(result.x[integer_vars][non_integer_var_index]), bounds_right[non_integer_var_index][1])
    subproblem_right = branch_and_bound(c, A_ub, b_ub, bounds_right, integer_vars, incumbent)

    # Check if both subproblems are pruned
    if subproblem_left is None and subproblem_right is None:
        print(f"Iteration {iteration_count}: Both subproblems are pruned. No further exploration.\n{'*' * 30}")
        return None

    # Return the best solution found in the current branch
    if subproblem_left is None:
        return subproblem_right
    elif subproblem_right is None:
        return subproblem_left
    elif subproblem_left[0] > subproblem_right[0]:
        return subproblem_left
    else:
        return subproblem_right

# Read data from CSV file
data = pd.read_csv('./ipt.csv')

# Extract coefficients and constraints from the data
c = data['c'].values
A_ub = data[['A1', 'A2']].values
b_ub = data['b'].values
bounds = list(zip(data['lb'], data['ub']))
integer_vars = data['integer'].astype(int).to_list()

# Solve the integer programming problem using Branch and Bound
optimal_solution = branch_and_bound(c, A_ub, b_ub, bounds, integer_vars)

# Print the final result
if optimal_solution is not None:
    print("\nOptimal value:", optimal_solution[0])
    print("Optimal solution:", optimal_solution[1])
else:
    print("\nNo feasible solution found.")
