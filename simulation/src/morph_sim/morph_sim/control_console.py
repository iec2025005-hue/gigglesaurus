"""Combined keyboard control with real key-up events and live simulation telemetry."""
import json,math,time,tkinter as tk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .timing import TimedVelocityPublisher
from std_msgs.msg import String,Bool
EVENTS={'e':'expand','x':'compact','n':'normal','h':'raise','l':'lower','1':'track','2':'rocky','3':'climb','4':'sand','k':'estop','space':'estop','u':'reset','v':'perception_toggle','f':'flip'}

def main(test_hook=None):
 rclpy.init();node=Node('morph_control_console');pub=TimedVelocityPublisher(node,'console');ev=node.create_publisher(String,'/morph/event',10);cam=node.create_publisher(Bool,'/morph/camera/show',1)
 root=tk.Tk();root.title('MORPH — combined controls');root.geometry('960x620');root.configure(bg='#101820');keys=set();pending={};state={};physics={};thermal={};show=[True];clock=[None,None,0.];active=[True];focused=[True]
 def data(target,m):
  try:target.update(json.loads(m.data))
  except ValueError:pass
 node.create_subscription(String,'/morph/status',lambda m:data(state,m),1)
 node.create_subscription(String,'/morph/physics/status',lambda m:data(physics,m),1)
 node.create_subscription(String,'/morph/suspension/telemetry',lambda m:data(thermal,m),1)
 tk.Label(root,text='MORPH  /  RESEARCH CONSOLE',font=('DejaVu Sans',23,'bold'),fg='#59d6d2',bg='#101820').pack(pady=16)
 info=tk.StringVar();tk.Label(root,textvariable=info,justify='left',anchor='w',font=('DejaVu Sans Mono',12),fg='#edf3f7',bg='#101820').pack(fill='x',padx=24)
 tk.Label(root,text='Hold W/S + Q/C + A/D together to drive, strafe and turn.\nPress E / X / N while driving to expand / compact / centre.\nH / L ride height     1–4 terrain mode     Shift precision\nB brake     Space / K emergency stop     U reset     P camera     V sensor adaptation\nControls work while this window has focus. Release keys to stop.',justify='left',font=('DejaVu Sans',12),fg='#a8baca',bg='#101820').pack(padx=24,pady=15,anchor='w')
 row=tk.Frame(root,bg='#101820');row.pack(fill='x',padx=24)
 def event(name):ev.publish(String(data=name))
 for label,name in [('EXPAND','expand'),('COMPACT','compact'),('NORMAL','normal'),('RAISE','raise'),('LOWER','lower'),('SENSOR AUTO','perception_toggle'),('RESET','reset'),('STOP','estop')]:
  tk.Button(row,text=label,command=lambda n=name:event(n),bg='#27414b' if name!='estop' else '#9d3636',fg='white',takefocus=0).pack(side='left',padx=3,pady=4)
 def press(e):
  key=e.keysym.lower();pub.mark_input(e.type.name+':'+key)
  if key in pending:root.after_cancel(pending.pop(key))
  first=key not in keys;keys.add(key)
  if first and key in EVENTS:event(EVENTS[key])
  publish_command()
  if first and key=='p':show[0]=not show[0];cam.publish(Bool(data=show[0]))
 def release(e):
  key=e.keysym.lower();pub.mark_input(e.type.name+':'+key)
  def remove():keys.discard(key);pending.pop(key,None);publish_command()
  pending[key]=root.after(8,remove) # cancel synthetic X11 autorepeat releases
 def unfocus(e):
  pub.mark_input('focus_loss')
  # Internal focus changes to a button do not release held drive keys.
  def check():
   focus=root.focus_displayof()
   focused[0]=focus is not None and focus.winfo_toplevel()==root
   if not focused[0]:keys.clear();pub.publish(Twist())
  root.after_idle(check)
 root.bind('<KeyPress>',press);root.bind('<KeyRelease>',release);root.bind('<FocusOut>',unfocus);root.bind('<FocusIn>',lambda e:focused.__setitem__(0,True))
 def close():active[0]=False;keys.clear();pub.publish(Twist());root.destroy()
 root.protocol('WM_DELETE_WINDOW',close)
 def publish_command():
  if not active[0] or not rclpy.ok():return
  msg=Twist();speed=.12 if 'shift_l' in keys or 'shift_r' in keys else .30
  if focused[0] and 'b' not in keys and 'space' not in keys and 'k' not in keys:
   msg.linear.x=speed*(int('w' in keys or 'up' in keys)-int('s' in keys or 'down' in keys));msg.linear.y=speed*(int('q' in keys)-int('c' in keys));msg.angular.z=.5*(int('a' in keys or 'left' in keys)-int('d' in keys or 'right' in keys))
  pub.publish(msg)
 last_draw=[0.];display_timing=node.create_publisher(String,'/morph/console/display_timing',1)
 def tick():
  if not active[0] or not rclpy.ok():return
  publish_command()
  for _ in range(3):rclpy.spin_once(node,timeout_sec=0.)
  root.after(8,tick)
  if time.monotonic()-last_draw[0]<.15:return
  last_draw[0]=time.monotonic()
  sim=physics.get('sim_time_s');wall=time.monotonic()
  if sim is not None and (clock[0] is None or wall-clock[1]>1.):
   if clock[0] is not None:clock[2]=(sim-clock[0])/(wall-clock[1])
   clock[0]=sim;clock[1]=wall
  power=physics.get('power',{});damper=thermal.get('temperature_C',[])
  actual=state.get('actual_speed') or 0.;temperatures=power.get('winding_C',[0.])
  lines=[f"{'READY' if state.get('feedback_ready') else 'WAITING FOR CONTROLLERS'}   Mode: {state.get('mode','—')}    Emergency stop: {state.get('estop',False)}",f"Command {state.get('command_speed',0):.2f} m/s    Actual {actual:.2f} m/s    RTF {clock[2]:.2f}",f"Chassis {1000*state.get('stage_offset',0):+.1f} → {1000*state.get('target_stage_offset',0):+.1f} mm per stage",f"Steering: {[round(a) for a in state.get('steering_measured_degrees',state.get('steering_degrees',[]))]} deg",f"Wheel contacts: {state.get('wheel_contacts',{})}",f"Battery model {power.get('battery_V',0):.2f} V  |  motor model max {max(temperatures):.1f} °C",f"Damper model °C: {', '.join(f'{v:.1f}' for v in damper) or 'waiting'}",f"Sector: {physics.get('zone','—')}    Input age: {state.get('input_age_sim_ms',0):.0f} sim ms",f"Sensor auto: {state.get('perception_auto',False)} | {state.get('perception',{}).get('reason','waiting for range data')}"]
  info.set('\n'.join(lines))
  display_timing.publish(String(data=json.dumps({'wall_ns':time.monotonic_ns(),'sim_ns':node.get_clock().now().nanoseconds,'scope':'Tk widget update time, not compositor/Gazebo rendered frame time','viewable':bool(root.winfo_viewable())})))
 root.after(8,tick);root.focus_force()
 if test_hook is not None:test_hook(root,node,close)
 try:root.mainloop()
 except KeyboardInterrupt:pass
 finally:
  if rclpy.ok():pub.publish(Twist())
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
