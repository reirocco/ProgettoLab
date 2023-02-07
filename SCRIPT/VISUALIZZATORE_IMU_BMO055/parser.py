import json

class Parser:
    # parsing 
    def parse(self,y:str):
        if y == None:
            y = "{}"
        return json.loads(y)