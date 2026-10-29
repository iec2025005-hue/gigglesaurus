#!/usr/bin/env bash
# Dedicated discovery partition: shutdown cannot target another project's server.
MORPH_TRANSPORT_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GZ_PARTITION="${MORPH_GZ_PARTITION:-morph_$(python3 -c 'import hashlib,sys;print(hashlib.sha256(sys.argv[1].encode()).hexdigest()[:12])' "$MORPH_TRANSPORT_WS")}"
