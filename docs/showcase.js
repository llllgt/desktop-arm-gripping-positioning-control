"use strict";
const D=window.ARM_RESULTS;
const $=id=>document.getElementById(id);
const num=(v,d=3)=>v==null?'—':Number(v).toFixed(d);
const dl=(id,items)=>{$(id).replaceChildren(...items.map(([name,value])=>{const row=document.createElement('div'),dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=name;dd.textContent=value;row.append(dt,dd);return row;}));};
let selected='m050_mu1.20_h40';
function chart(id,trace,keys,colors,floor){
  const el=$(id),w=540,h=220,x0=48,y0=18,pw=470,ph=163;
  if(!trace.length){el.innerHTML='<text x="50" y="100">预检拒绝，无执行曲线</text>';return;}
  const maxT=trace.at(-1)[0],values=trace.flatMap(r=>keys.map(k=>r[k]));
  let low=Math.min(0,...values),high=Math.max(...values,floor??0)*1.12;
  if(high<=low)high=low+1;
  const px=t=>x0+t/maxT*pw,py=v=>y0+ph-(v-low)/(high-low)*ph;
  let svg='';
  for(let i=0;i<=4;i++){let v=low+(high-low)*i/4,y=py(v);svg+=`<line x1="${x0}" y1="${y}" x2="${x0+pw}" y2="${y}" stroke="#e2e9e4"/><text x="${x0-7}" y="${y+4}" text-anchor="end">${v.toFixed(1)}</text>`;}
  for(let i=0;i<=4;i++){let t=maxT*i/4;svg+=`<text x="${px(t)}" y="${y0+ph+22}" text-anchor="middle">${t.toFixed(1)}</text>`;}
  if(floor!=null)svg+=`<line x1="${x0}" y1="${py(floor)}" x2="${x0+pw}" y2="${py(floor)}" stroke="#a4afa8" stroke-dasharray="5 5"/><text x="${x0+pw-5}" y="${py(floor)-5}" text-anchor="end">初始中心高度 ${floor} mm</text>`;
  keys.forEach((k,i)=>{svg+=`<polyline fill="none" stroke="${colors[i]}" stroke-width="2" points="${trace.map(r=>`${px(r[0]).toFixed(2)},${py(r[k]).toFixed(2)}`).join(' ')}"/>`;});
  el.innerHTML=svg;
}
function showCase(c){
  selected=c.case;
  $('case-name').textContent=c.case;
  $('case-title').textContent=`${c.mass_g} g / μ ${c.friction} / ${c.clearance_mm} mm`;
  const badge=document.createElement('span');badge.className='badge '+(c.status==='passed'?'':'fail');badge.textContent=c.status==='passed'?'满足搬运标准':c.status==='rejected'?'预检拒绝':'未满足搬运标准';$('case-status').replaceChildren(badge);
  dl('case-metrics',[
    ['最终工件位置偏差',num(c.placement_error_mm)+' mm'],['最大抬升',num(c.maximum_lift_mm)+' mm'],
    ['TCP 跟踪 RMS',num(c.tcp_rmse_mm)+' mm'],['峰值关节力矩',c.peak_joint_torque_nm?num(Math.max(...c.peak_joint_torque_nm))+' N·m':'—'],
    ['实际接触滑动摩擦',c.measured_pad_friction_range?c.measured_pad_friction_range.map(v=>num(v,2)).join(' – '):'未检测到垫接触'],
    ['搬运期间双侧有效垫接触',num(c.bilateral_transport_contact_fraction*100,1)+'%'],
    ['金属夹指与工件接触步数',c.metal_workpiece_contact_steps??'—']]);
  const reasons={placement_error:'放置偏差超限',insufficient_lift:'抬升不足',fixture_collision:'机械臂与托座/地面接触',preflight_rejection:'路径预检未通过'};
  $('failure').textContent=c.failure_reasons.length?'未满足项：'+c.failure_reasons.map(r=>reasons[r]||r).join('、'):'满足当前判据；结果来自单次接触仿真。';
  chart('height-chart',c.trace,[1],['#146b62'],65);
  chart('force-chart',c.trace,[2,3],['#146b62','#ad7442']);
  document.querySelectorAll('.heatmap-table button').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.case===selected)));
}
function updateExperiments(){
  const height=Number($('clearance').value),status=$('status').value,atHeight=D.cases.filter(c=>c.clearance_mm===height);
  const shown=atHeight.filter(c=>status==='all'||c.status===status);
  $('filter-count').textContent=`显示 ${shown.length} / ${atHeight.length} 组，满足标准 ${atHeight.filter(c=>c.status==='passed').length} 组`;
  const table=document.createElement('table');table.className='heatmap-table';
  table.innerHTML='<caption class="sr-only">负载与滑动摩擦工况。按回车选择单元格</caption><thead><tr><th>负载 / g<br>摩擦 →</th>'+[.05,.2,.6,1.2].map(mu=>`<th>${mu}</th>`).join('')+'</tr></thead>';
  const body=document.createElement('tbody');
  [20,50,120,200].forEach(m=>{const row=document.createElement('tr'),th=document.createElement('th');th.scope='row';th.textContent=m;row.append(th);
    [.05,.2,.6,1.2].forEach(mu=>{const c=atHeight.find(r=>r.mass_g===m&&r.friction===mu),td=document.createElement('td'),b=document.createElement('button');b.className=(c.status==='passed'?'pass':'fail')+(status!=='all'&&status!==c.status?' dim':'');b.dataset.case=c.case;b.type='button';b.textContent=c.status==='passed'?'通过':'未通过';b.setAttribute('aria-label',`${m} 克，摩擦 ${mu}，${height} 毫米，${b.textContent}`);b.setAttribute('aria-pressed',String(c.case===selected));b.onclick=()=>showCase(c);td.append(b);row.append(td);});body.append(row);});
  table.append(body);$('heatmap').replaceChildren(table);
  $('results-table').replaceChildren(...shown.map(c=>{const row=document.createElement('tr');[c.mass_g,c.friction,c.status==='passed'?'满足标准':'未满足标准',num(c.placement_error_mm),num(c.maximum_lift_mm)].forEach(v=>{const td=document.createElement('td');td.textContent=v;row.append(td);});return row;}));
  const current=atHeight.find(c=>c.case===selected)||atHeight.find(c=>c.status==='passed')||atHeight[0];showCase(current);
}
function structure(){const c=D.structure.cases.find(c=>c.thickness_mm===Number($('thickness').value));dl('structure-metrics',[
  ['单侧铝背板质量',num(c.carrier_mass_g)+' g'],['梁理论端部挠度',num(c.beam_deflection_mm,4)+' mm'],
  ['根部名义弯曲应力',num(c.nominal_root_stress_mpa,2)+' MPa'],['名义屈服安全系数',num(c.nominal_yield_factor,2)]]);}
function workspace(){let s='';const x0=65,y0=20,pw=430,ph=285;
  D.workspace.cells.forEach(([x,y,reachable])=>{s+=`<rect x="${x0+(x-.08)/.20*pw-4}" y="${y0+(.15-y)/.30*ph-2}" width="8" height="4" fill="${reachable?'#b2d6c6':'#e5eae6'}"/>`;});
  [[.17,-.05,'取料'],[.17,.06,'放置']].forEach(([x,y,label])=>{const px=x0+(x-.08)/.20*pw,py=y0+(.15-y)/.30*ph;s+=`<circle cx="${px}" cy="${py}" r="5" fill="#173b3b"/><text x="${px+8}" y="${py+4}">${label}</text>`;});
  for(let i=0;i<=4;i++){s+=`<text x="${x0+pw*i/4}" y="330" text-anchor="middle">${80+50*i}</text>`;}
  for(let i=0;i<=6;i++){s+=`<text x="48" y="${y0+ph*i/6+4}" text-anchor="end">${150-50*i}</text>`;}
  s+='<text x="480" y="346">X / mm</text><text x="5" y="14">Y / mm</text>';$('workspace-chart').innerHTML=s;
  $('workspace-note').textContent=`高度 ${D.workspace.height_m*1000} mm / 俯仰 ${D.workspace.pitch_rad} rad / ${D.workspace.reachable_cells} 个可达单元，共 ${D.workspace.cells.length} 个`;
}
if(D){
  $('default-error').textContent=num(D.default.placement_error_mm);
  $('pass-count').textContent=D.counts.passed+' / '+D.cases.length;
  $('revision').textContent=D.revision;
  dl('preflight-metrics',[['检查状态',D.preflight.feasible?'通过':'未通过'],['采样配置数',D.preflight.samples],
    ['最小关节限位余量',num(D.preflight.minimum_joint_margin_rad)+' rad'],['位置雅可比最大条件数',num(D.preflight.maximum_position_jacobian_condition)],
    ['规划持续时间',num(D.preflight.planned_duration_s)+' s']]);
  $('clearance').onchange=updateExperiments;$('status').onchange=updateExperiments;$('thickness').onchange=structure;
  updateExperiments();structure();workspace();
}else{$('filter-count').textContent='实验数据未加载，请通过完整 JSON 链接查看结果。';}
