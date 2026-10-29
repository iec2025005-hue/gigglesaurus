"""Shared geometry for the connected course. Bounds are in world metres."""
import json
from pathlib import Path
from types import SimpleNamespace

MODES={'smooth':'track','grass':'grass','sand':'sand','mud':'mud','ramps':'climb','debris':'debris','rocks':'rocky','snow':'snow','mixed':'mixed','village':'grass','pipeline':'track','warehouse':'track','rubble':'debris'}

class Course:
    def __init__(self,share):
        self.data=json.loads((Path(share)/'config/sectors.json').read_text())
        self.sectors=[SimpleNamespace(**s,name=s['id'],mode=MODES[s['id']]) for s in self.data['sectors']]
    def at(self,x,y):
        for sector in self.sectors:
            a,b,c,d=sector.bounds
            if a<=x<b and c<=y<d:return sector
        return None
    def spawn(self,name):
        if name=='start':return self.data['default_spawn']
        for sector in self.sectors:
            if sector.name==name:return sector.spawn
        raise ValueError('Unknown sector '+name+'. Choose start or '+', '.join(s.name for s in self.sectors))
