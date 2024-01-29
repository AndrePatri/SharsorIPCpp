class NamingConventions():

    def __init__(self):

        self.data_basename = "Data"

        self.n_rows_basename = "NRows"

        self.n_cols_basename = "NCols"

        self.dtype_basename = "DType"

    def DataName(self, 
            namespace: str, 
            basename: str):
        
        return namespace + "/" + \
            basename + "/" + \
            self.data_basename
    
    def nRowsName(self, 
            namespace: str, 
            basename: str):
        
        return namespace + "/" + \
            basename + "/" + \
            self.n_rows_basename
    
    def nColsName(self, 
            namespace: str, 
            basename: str):

        return namespace + "/" + \
            basename + "/" + \
            self.n_cols_basename
    
    def dTypeName(self, 
            namespace: str, 
            basename: str):

        return namespace + "/" + \
            basename + "/" + \
            self.dtype_basename


