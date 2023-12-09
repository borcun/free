""" the module is used to parse csv file that include input parameters for the project """

import pandas as pd

"""
The function that parses csv file, then get input parameter inside

Params:
  path - csv file path
Return:
  coefficients if succeed, otherwise None
"""
def parse(path):
    try:
        df = pd.read_csv(path)
        coefficients = []

        for row in range(len(df)):
            coefficients.append(df.iloc[row].to_list())

        return coefficients
    except:
        return None
