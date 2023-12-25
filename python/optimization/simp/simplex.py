import csv

class Simplex:
    def __init__(self):
        pass

    @staticmethod
    def getEquations(path):
        with open(path, 'r') as file:
            reader = csv.reader(file)
            data = [list(map(float, row)) for row in reader]

        return data

    def execute(self, path, max_iterations=1000, tolerance=1e-10):
        data = self.getEquations(path)

        if data is None:
            return False

        A = [row[:-1] for row in data]
        b = [row[-1] for row in data]
        c = data[-1][:-1]

        m = len(b)
        n = len(c)

        tableau = [[0] * (n + m + 1) for _ in range(m + 1)]

        for i in range(m):
            tableau[i][:n] = A[i][:]
            tableau[i][n + i] = 1
            tableau[i][-1] = b[i]

        tableau[-1][:n] = [-val for val in c]

        iteration = 0
        while any(val < -tolerance for val in tableau[-1][:-1]) and iteration < max_iterations:
            pivot_col = min(range(n), key=lambda i: tableau[-1][i])
            ratios = [(tableau[i][-1] / tableau[i][pivot_col], i) for i in range(m) if tableau[i][pivot_col] > tolerance]

            if not ratios:
                raise ValueError("Linear programming problem is unbounded.")

            pivot_row = min(ratios, key=lambda x: x[0])[1]
            pivot_element = tableau[pivot_row][pivot_col]

            for i in range(m + 1):
                tableau[i][pivot_col] /= pivot_element

            for i in range(m + 1):
                if i != pivot_row:
                    multiplier = tableau[i][pivot_col]
                    for j in range(n + m + 1):
                        tableau[i][j] -= multiplier * tableau[pivot_row][j]

            iteration += 1

        if iteration == max_iterations:
            raise ValueError("Maximum number of iterations reached. The algorithm may not have converged.")

        print("objective_value:", -tableau[-1][-1], "variables:", tableau[-1][:n])
        return True