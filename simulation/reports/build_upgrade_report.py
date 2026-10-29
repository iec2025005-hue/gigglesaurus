from pathlib import Path
import sys,json,html
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,PageBreak,KeepTogether
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from report_content import PAGES
P=Path(__file__).resolve().parents[1];d=json.loads((P/'validation/performance/summary.json').read_text());out=P/'output/pdf/MORPH_Project_Review_and_Upgrade_Plan.pdf'
font='/usr/share/fonts/truetype/dejavu/';pdfmetrics.registerFont(TTFont('DV',font+'DejaVuSans.ttf'));pdfmetrics.registerFont(TTFont('DVB',font+'DejaVuSans-Bold.ttf'))
ink=colors.HexColor('#162b36');accent=colors.HexColor('#147b7a');muted=colors.HexColor('#53636e');light=colors.HexColor('#edf3f4')
styles={'body':ParagraphStyle('body',fontName='DV',fontSize=9.2,leading=13.5,textColor=ink,spaceAfter=8),'sub':ParagraphStyle('sub',fontName='DVB',fontSize=10.6,leading=14,textColor=accent,spaceBefore=9,spaceAfter=4),'title':ParagraphStyle('title',fontName='DVB',fontSize=22,leading=27,textColor=ink,spaceAfter=12),'small':ParagraphStyle('small',fontName='DV',fontSize=7.8,leading=11,textColor=muted),'cell':ParagraphStyle('cell',fontName='DV',fontSize=7.8,leading=10,textColor=ink)}
def para(s,kind='body'):return Paragraph(html.escape(s),styles[kind])
def footer(c,doc):
 c.setStrokeColor(colors.HexColor('#cfdbdf'));c.line(42,41,553,41);c.setFont('DV',7);c.setFillColor(muted);c.drawString(42,28,'MORPH  /  engineering review  /  28 October 2026');c.drawRightString(553,28,f'{doc.page} / 8')
labels={'gui_2ms':'Visible / previous camera','gui_balanced_2ms':'Visible / balanced','headless_2ms':'Headless / 2 ms','headless_1ms':'Headless / 1 ms','gui_balanced_soak':'Visible / longer soak','gui_realtime':'Visible / low-cost preset','gui_realtime_lp2':'Visible / 2 render workers'}
flow=[]
for i,(title,sections) in enumerate(PAGES):
 if i:flow.append(PageBreak())
 flow.append(para(f'{i+1:02d}  /  MORPH PROJECT REVIEW','small'));flow.append(Spacer(1,8));flow.append(para(title,'title'))
 if i==0:
  flow.append(para('Measured simulation performance • implementation audit • staged upgrade plan','small'));flow.append(Spacer(1,10))
 for j,(head,body) in enumerate(sections):
  flow.extend([para(head,'sub'),para(body)])
  if i==0 and j==1:
   rows=[['Configuration','Sim time\n(s)','Mean\nRTF','1 s RTF\np05*','Response\n(ms)']]
   for r in d['cases']:rows.append([labels.get(r['case'],r['case']),f"{r['duration_sim_s']:.1f}",f"{r['aggregate_rtf']:.3f}",f"{r['sampled_1s_rtf_p05']:.3f}",f"{r['response_wall_ms']:.0f}"])
   cells=[[para(t,'cell') for t in row] for row in rows];table=Table(cells,colWidths=[212,65,67,76,87],repeatRows=1,hAlign='LEFT');table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),light),('LINEBELOW',(0,0),(-1,0),.7,accent),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),('LINEBELOW',(0,1),(-1,-1),.3,colors.HexColor('#dce5e8'))]));flow.extend([Spacer(1,5),table,Spacer(1,5),para('*Sampled/interpolated application-level estimate; one response trial per case. Source: local validation/performance JSON traces, 28 October 2026.','small')])
doc=SimpleDocTemplate(str(out),pagesize=(595.28,841.89),rightMargin=42,leftMargin=42,topMargin=40,bottomMargin=53,title='MORPH Project Review and Upgrade Plan',author='MORPH project engineering review');doc.build(flow,onFirstPage=footer,onLaterPages=footer)
md=['# MORPH Project Review and Upgrade Plan','28 October 2026','']
for title,sections in PAGES:
 md+=['## '+title,'']
 for h,b in sections:md+=['### '+h,'',b,'']
md+=['## Measured results','']
for r in d['cases']:md+=[f"- {labels.get(r['case'],r['case'])}: {r['duration_sim_s']:.1f} simulated seconds; mean RTF {r['aggregate_rtf']:.3f}; sampled one-second p05 {r['sampled_1s_rtf_p05']:.3f}; forward response {r['response_wall_ms']:.0f} ms."]
(P/'reports/MORPH_Project_Review_and_Upgrade_Plan.md').write_text('\n'.join(md)+'\n');print(out)
