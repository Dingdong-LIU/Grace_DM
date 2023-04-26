import pandas as pd

class database_reader():
    def __init__(self, filename) -> None:
        super().__init__()
        self.df = pd.read_excel(filename)
    
    def lookup_table(self, df:pd.DataFrame, intent_name:str):
        df_line = self.df[self.df["Intent Name"] == intent_name]
        lookup_result = {}

        lookup_result['expressions'] = df_line["POSES"].item()
        lookup_result['exp_start'] = df_line["E_Start (%)"].item()/100
        lookup_result['exp_end'] = df_line["E_End (%)"].item()/100
        lookup_result['exp_mag'] = df_line["Magnitude"].item()
        return lookup_result