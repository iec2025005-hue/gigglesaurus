"""Local monotonic timing and timestamped velocity command transport."""
from functools import wraps
import json,math,socket,time,uuid


def timed_callback(method):
    @wraps(method)
    def wrapped(self,*args,**kwargs):
        start=time.monotonic_ns()
        try:return method(self,*args,**kwargs)
        finally:
            duration=time.monotonic_ns()-start
            stats=self.callback_timing.setdefault(method.__name__,{'count':0,'total_wall_ns':0,'max_wall_ns':0,'last_wall_ns':0})
            stats['count']+=1;stats['total_wall_ns']+=duration;stats['max_wall_ns']=max(stats['max_wall_ns'],duration);stats['last_wall_ns']=duration
    return wrapped


def decode_command(payload,now_sim_ns,now_wall_ns,timeout_s=.2,future_s=.02):
    data=json.loads(payload)
    values=data['values']
    if len(values)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):raise ValueError('Expected three finite velocities')
    for key in ('source_sim_ns','source_wall_ns','sequence'):
        if type(data[key]) is not int or data[key]<0:raise ValueError('Invalid command timestamp or sequence')
    if not isinstance(data['source_id'],str) or not data['source_id']:raise ValueError('Missing command source identity')
    if not -future_s*1e9<=now_sim_ns-data['source_sim_ns']<=timeout_s*1e9:raise ValueError('Stale/future source command')
    if data.get('host')==socket.gethostname() and not -future_s*1e9<=now_wall_ns-data['source_wall_ns']<=timeout_s*1e9:raise ValueError('Stale local wall-clock source command')
    return data


class TimedVelocityPublisher:
    """Publisher-compatible console path; standard /cmd_vel remains accepted."""
    def __init__(self,node,source='console'):
        from std_msgs.msg import String
        self.node=node;self.message_type=String;self.pub=node.create_publisher(String,'/morph/cmd_vel_timed',1)
        self.source_id=source+'_'+uuid.uuid4().hex;self.sequence=0;self.input_event=None
    def mark_input(self,event):
        self.input_event={'event':event,'wall_ns':time.monotonic_ns(),'sim_ns':self.node.get_clock().now().nanoseconds}
    def publish(self,message):
        self.sequence+=1
        data={'values':[message.linear.x,message.linear.y,message.angular.z], 'source_id':self.source_id,'sequence':self.sequence,'host':socket.gethostname(),'source_sim_ns':self.node.get_clock().now().nanoseconds,'source_wall_ns':time.monotonic_ns(),'input_event':self.input_event}
        self.pub.publish(self.message_type(data=json.dumps(data,allow_nan=False)))
