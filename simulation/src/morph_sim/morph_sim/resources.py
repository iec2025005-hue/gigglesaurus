"""Validate explicit SDF resource references without network downloads."""
from pathlib import Path
from urllib.parse import unquote, urlparse


def validate_local_resources(tree,share):
    share=Path(share)
    paths=set()
    for node in tree.iter():
        if node.tag not in ('uri','albedo_map','normal_map','roughness_map','metalness_map','emissive_map','environment_map') or not node.text:
            continue
        uri=node.text.strip()
        if uri.startswith('model://'):
            resource=uri[len('model://'):];model,_,rest=resource.partition('/')
            path=(share if model=='morph_sim' else share/'models'/model)/rest
        elif uri.startswith('file://'):path=Path(unquote(urlparse(uri).path))
        elif '://' in uri:raise ValueError('Non-local resource is not permitted: '+uri)
        else:path=share/uri
        if not path.exists():raise ValueError('Missing local resource: '+uri)
        paths.add(str(path.resolve()))
    return sorted(paths)
