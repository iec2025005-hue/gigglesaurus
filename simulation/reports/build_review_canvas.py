from pathlib import Path
import json
from report_content import PAGES
P=Path(__file__).resolve().parents[1];d=json.loads((P/'validation/performance/summary.json').read_text())
s='''import { useHostTheme, useState } from "cursor/canvas";
const cases = DATA;
const sections: [string, [string,string][]][] = SECTIONS;
const labels: Record<string,string> = {gui_2ms:"Visible / previous camera",gui_balanced_2ms:"Visible / balanced",headless_2ms:"Headless / 2 ms",headless_1ms:"Headless / 1 ms",gui_balanced_soak:"Visible / longer soak",gui_realtime:"Visible / low-cost preset",gui_realtime_lp2:"Visible / 2 render workers"};
export default function MorphReview(){
 const t=useHostTheme();const [selected,setSelected]=useState(0);
 const measured=cases.find(r=>r.case==="gui_realtime_lp2");
 const line={borderBottom:`1px solid ${t.stroke.secondary}`};
 return <main style={{fontFamily:"system-ui",color:t.text.primary,background:t.bg.editor,padding:28,maxWidth:1120,margin:"auto",lineHeight:1.55}}>
  <p style={{fontSize:12,color:t.text.tertiary,letterSpacing:1}}>MORPH / ENGINEERING REVIEW / 28 OCTOBER 2026</p>
  <h1 style={{fontSize:24,marginBottom:8}}>Performance, remaining work and upgrade plan</h1>
  <p style={{color:t.text.secondary,maxWidth:900}}>The low-cost visible preset demonstrated near-real-time simulation: 0.998 mean RTF over 208.5 simulated seconds. Repeated latency testing, long-run acceptance and hardware calibration remain open. The table contains measurements, while the roadmap contains proposed work.</p>
  <div style={{display:"flex",gap:36,padding:"18px 0",flexWrap:"wrap",...line}}>
   {measured && <div><strong style={{color:t.accent.primary,fontSize:22}}>{measured.aggregate_rtf.toFixed(3)}</strong><div>mean RTF / low-cost visible preset</div></div>}
   <div><strong style={{color:t.accent.primary,fontSize:22}}>51</strong><div>software checks passed</div></div>
   <div><strong style={{color:t.accent.primary,fontSize:22}}>13</strong><div>safe entrances checked</div></div>
   <div><strong style={{fontSize:22}}>Assumed</strong><div>physical parameters awaiting calibration</div></div>
  </div>
  <h2 style={{fontSize:18,marginTop:24}}>Measured performance by configuration</h2>
  <div style={{overflowX:"auto"}}><table style={{width:"100%",borderCollapse:"collapse",fontSize:13}}>
   <thead><tr>{["Configuration","Simulated duration (s)","Mean RTF","Sampled 1 s RTF p05","Forward response (ms)"].map(x=><th key={x} style={{textAlign:"left",padding:10,background:t.fill.tertiary,...line}}>{x}</th>)}</tr></thead>
   <tbody>{cases.map(r=><tr key={r.case}>{[labels[r.case]||r.case,r.duration_sim_s.toFixed(1),r.aggregate_rtf.toFixed(3),r.sampled_1s_rtf_p05?.toFixed(3),r.response_wall_ms.toFixed(0)].map((v,i)=><td key={i} style={{padding:10,...line}}>{v}</td>)}</tr>)}</tbody>
  </table></div>
  <p style={{fontSize:12,color:t.text.tertiary}}>Source: local validation/performance JSON traces, 28 October 2026. RTF = simulation seconds / wall seconds. One response trial per case; p95 latency is unestablished. One-second RTF values are interpolated from approximately 10 Hz application samples. Sequential runs do not establish causal improvement.</p>
  <h2 style={{fontSize:18,marginTop:28}}>Review and implementation roadmap</h2>
  <nav style={{display:"flex",gap:6,flexWrap:"wrap",marginBottom:22}}>{sections.map(([name],i)=><button key={name} onClick={()=>setSelected(i)} style={{fontSize:12,padding:"8px 11px",border:`1px solid ${t.stroke.secondary}`,borderRadius:4,background:selected===i?t.accent.control:t.fill.tertiary,color:selected===i?t.text.onAccent:t.text.primary,cursor:"pointer"}}>{name}</button>)}</nav>
  <section><h2 style={{fontSize:20}}>{sections[selected][0]}</h2>{sections[selected][1].map(([heading,body])=><article key={heading} style={{padding:"10px 0 16px",...line}}><h3 style={{fontSize:15,color:t.accent.primary,margin:"0 0 6px"}}>{heading}</h3><p style={{fontSize:14,margin:0,color:t.text.secondary}}>{body}</p></article>)}</section>
  <p style={{fontSize:12,color:t.text.tertiary,marginTop:24}}>Proposed release gate: mean RTF ≥ 0.95, sampled one-second RTF p05 ≥ 0.90, response p95 ≤ 250 ms over at least 20 trials, 5 repeat runs and a 10-minute loaded soak. Current measurements provide baseline evidence; they do not complete this gate.</p>
 </main>;
}
'''.replace('DATA',json.dumps(d['cases'])).replace('SECTIONS',json.dumps(PAGES))
Path('/tmp/morph-project-review.canvas.tsx').write_text(s)
