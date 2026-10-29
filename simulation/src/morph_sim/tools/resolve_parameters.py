#!/usr/bin/env python3
"""Regenerate the compatibility YAML after an intentional schema edit."""
from pathlib import Path
import sys
import yaml
P=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(P))
from morph_sim.parameters import resolve_schema
params,_=resolve_schema(P/'config/parameter_schema.yaml')
(P/'config/params.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
print('Generated params.yaml from parameter_schema.yaml; all uncertainties/provenance retained in schema.')
