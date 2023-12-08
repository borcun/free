import pandas as pd

def parse(path):
    try:
        df = pd.read_csv(path)
        coefficients = []

        for row in range(len(df)):
            coefficients.append(df.iloc[row].to_list())

        return coefficients
    except:
        return None
