from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from sequence_gaps import SequenceGaps


def test_sequence_gaps_ignore_startup_and_separate_publishers():
    gaps=SequenceGaps()
    for seq in (100,101,104,104,103,105):gaps.observe(dict(publication_sequence_number=seq,publisher_gid={'data':b'a'}))
    gaps.observe(dict(publication_sequence_number=400,publisher_gid={'data':b'b'}))
    report=gaps.report()
    assert report['missing_writer_sequences']==2
    assert report['publishers'][b'a'.hex()]['duplicate_or_reordered']==2
    assert report['publishers'][b'b'.hex()]['missing_sequences']==0


def test_missing_metadata_is_unknown_not_zero_drops():
    gaps=SequenceGaps();gaps.observe(None);gaps.observe({'publication_sequence_number':None})
    assert gaps.report()['missing_writer_sequences'] is None
    assert gaps.report()['samples_without_sequence_metadata']==2
