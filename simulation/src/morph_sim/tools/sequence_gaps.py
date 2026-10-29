"""Observed ROS writer-sequence gaps, scoped to the bridge-to-observer hop."""
class SequenceGaps:
    def __init__(self):self.streams={};self.unsupported=0
    def observe(self,info):
        sequence=info.get('publication_sequence_number') if info else None
        gid=info.get('publisher_gid') if info else None
        key=bytes(gid.get('data',b'')).hex() if isinstance(gid,dict) else ''
        if type(sequence) is not int or sequence<0 or not key:self.unsupported+=1;return
        stream=self.streams.setdefault(key,{'first':sequence,'last':sequence,'messages':0,'missing_sequences':0,'duplicate_or_reordered':0})
        if stream['messages']:
            if sequence>stream['last']:
                stream['missing_sequences']+=sequence-stream['last']-1;stream['last']=sequence
            else:stream['duplicate_or_reordered']+=1
        stream['messages']+=1
    def report(self):
        return {'missing_writer_sequences':sum(s['missing_sequences'] for s in self.streams.values()) if self.streams else None,'samples_without_sequence_metadata':self.unsupported,'publishers':self.streams,'scope':'Observed ROS publisher-writer gaps after first observed sequence, per GID; excludes unobserved startup and loss inside Gazebo or the bridge. Reordering can resemble loss; gaps are not certified physical sensor drops.'}
