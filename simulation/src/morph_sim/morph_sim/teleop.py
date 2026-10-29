"""Pygame key-state teleop: supports genuine simultaneous key presses."""
import json,time
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
EVENTS={pygame.K_1:'track',pygame.K_2:'rocky',pygame.K_3:'climb',pygame.K_4:'sand',pygame.K_x:'compact',pygame.K_n:'normal',pygame.K_e:'expand',pygame.K_PAGEUP:'raise',pygame.K_PAGEDOWN:'lower',pygame.K_TAB:'flip',pygame.K_SPACE:'estop',pygame.K_r:'reset',pygame.K_t:'auto'}
def main():
 rclpy.init();node=Node('morph_teleop');pub=node.create_publisher(Twist,'/cmd_vel',10);ev=node.create_publisher(String,'/morph/event',10);status={}
 def on_status(m):
  nonlocal status
  try:status=json.loads(m.data)
  except ValueError:pass
 node.create_subscription(String,'/morph/status',on_status,10)
 pygame.init();screen=pygame.display.set_mode((920,520));pygame.display.set_caption('MORPH | Swerve terrain console');font=pygame.font.SysFont('DejaVu Sans Mono',19);title=pygame.font.SysFont('DejaVu Sans',34,bold=True);clock=pygame.time.Clock();running=True
 try:
  while running and rclpy.ok():
   rclpy.spin_once(node,timeout_sec=0.)
   for event in pygame.event.get():
    if event.type==pygame.QUIT:running=False
    elif event.type==pygame.KEYDOWN:
     if event.key==pygame.K_ESCAPE:running=False
     elif event.key in EVENTS:ev.publish(String(data=EVENTS[event.key]))
   keys=pygame.key.get_pressed();msg=Twist();focused=pygame.key.get_focused()
   if focused:
    forward=int(keys[pygame.K_w] or keys[pygame.K_UP])-int(keys[pygame.K_s] or keys[pygame.K_DOWN]);turn=int(keys[pygame.K_a] or keys[pygame.K_LEFT])-int(keys[pygame.K_d] or keys[pygame.K_RIGHT]);strafe=int(keys[pygame.K_q])-int(keys[pygame.K_c]);scale=.3 if keys[pygame.K_LSHIFT] else .65
    if not keys[pygame.K_b]:msg.linear.x=scale*forward;msg.linear.y=scale*strafe;msg.angular.z=.7*turn
   pub.publish(msg);screen.fill((10,20,31));screen.blit(title.render('MORPH / TERRAIN CONSOLE',True,(49,220,224)),(28,24))
   lines=[f"MODE {status.get('mode','waiting')}    ESTOP {status.get('estop',False)}    AUTO {status.get('auto',False)}",f"Speed cmd {status.get('command_speed',0.):.2f} m/s | actual {status.get('actual_speed')} | feedback {status.get('feedback_ready',False)}",f"Steering deg {str([round(x,1) for x in status.get('steering_degrees',[])])}",f"Stage offset {1000*status.get('stage_offset',0.):.1f} mm | target {1000*status.get('target_stage_offset',0.):.1f} mm",f"Roll {status.get('roll_deg',0.):.1f} deg | Pitch {status.get('pitch_deg',0.):.1f} deg",'WASD / arrows drive + turn | Q/C strafe | SHIFT slow | B brake','1 track  2 rocky  3 climb  4 sand | T tilt-assist heuristic','X compact  N normal  E expand | PgUp/PgDn ride height','TAB stopped 180-degree sweep | SPACE latch stop | R reset','Release keys to brake. Focus loss sends zero. ESC quits.']
   for i,line in enumerate(lines):screen.blit(font.render(line,True,(232,241,247) if i<5 else (155,184,202)),(28,88+38*i))
   pygame.display.flip();clock.tick(30)
 finally:
  for _ in range(3):pub.publish(Twist());time.sleep(.02)
  pygame.quit();node.destroy_node();
  if rclpy.ok():rclpy.shutdown()
