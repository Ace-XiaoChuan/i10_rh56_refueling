/* 单组静态构型查看：不插值，不生成执行指令。 */
const names=['拇指','食指','中指'],colors=['#ef681b','#2074d4','#1aa25c'];
let view='overview',reversed=false;
const el=id=>document.getElementById(id);
function scatter(p,name,color,size){return {type:'scatter3d',mode:'markers',name,x:p.map(v=>v[0]),y:p.map(v=>v[1]),z:p.map(v=>v[2]),marker:{color,size}}}
function draw(){
 const f=DATA.frames[Number(el('state').value)],tr=[];
 if(el('env').checked)tr.push({...scatter(DATA.environment,'实测环境','#a6b1bb',1.5),opacity:.6});
 tr.push(scatter(DATA.target,'实测握梁','#dbac42',2));
 if(el('assumed').checked)tr.push({...scatter(DATA.assumed,'人工假设侧面','#9e63bd',1.7),opacity:.45});
 if(el('hand').checked)DATA.meshes.forEach((mesh,j)=>{const t=f.poses[j],p=mesh.vertices.map(v=>[0,1,2].map(r=>t[r][0]*v[0]+t[r][1]*v[1]+t[r][2]*v[2]+t[r][3]));
  tr.push({type:'mesh3d',name:mesh.link,x:p.map(v=>v[0]),y:p.map(v=>v[1]),z:p.map(v=>v[2]),i:mesh.faces.map(v=>v[0]),j:mesh.faces.map(v=>v[1]),k:mesh.faces.map(v=>v[2]),color:'#738da2',opacity:.88,showlegend:false});});
 DATA.targets.forEach((t,j)=>{
  const p=f.points[j],label=names[j];
  tr.push({...scatter([p],label+'指面',colors[j],6),mode:'markers+text',text:[label],textposition:'top center'});
  tr.push({...scatter([t],label+'参考目标',colors[j],4),marker:{color:colors[j],size:4,symbol:'diamond'}});
  tr.push({type:'scatter3d',mode:'lines',x:[p[0],t[0]],y:[p[1],t[1]],z:[p[2],t[2]],line:{color:colors[j],width:5,dash:'dash'},showlegend:false});
 });
 const close=view==='close';
 const scene={aspectmode:'data',xaxis:{title:'握梁长轴 / mm',range:close?[-40,40]:[-90,60]},yaxis:{title:'横向 / mm',range:close?[-35,35]:[-60,115]},zaxis:{title:'高度 / mm',range:close?[-24,24]:[-55,200]},camera:{eye:{x:1.35,y:reversed?-1.8:1.8,z:.8},center:{x:0,y:0,z:0}}};
 Plotly.react('plot',tr,{scene,margin:{l:0,r:0,t:5,b:0},legend:{orientation:'h'},uirevision:view+reversed},{responsive:true});
 el('status').textContent=Number(el('state').value)===2?
  `本组${DATA.passes?'满足当前静态几何条件':'尚未满足条件'}：三指距离 ${f.distances_mm.map(v=>v.toFixed(2)).join(' / ')} mm，未检出超容差侵入。`:
  `${f.title}：三指距离 ${f.distances_mm.map(v=>v.toFixed(2)).join(' / ')} mm；${f.geometry_pass?'未检出超容差侵入':`存在侵入，最大超出容差 ${f.maximum_excess_mm.toFixed(3)} mm`}。`;
}
el('metrics').innerHTML=names.map((name,j)=>`<tr><td>${name}</td><td>${DATA.frames[1].distances_mm[j].toFixed(2)} mm</td><td>${DATA.frames[2].distances_mm[j].toFixed(2)} mm</td><td>${DATA.frames[2].angles_deg[j].toFixed(2)}°</td></tr>`).join('');
el('state').onchange=draw;['hand','env','assumed'].forEach(id=>el(id).onchange=draw);
el('overview').onclick=()=>{view='overview';draw()};el('close').onclick=()=>{view='close';draw()};el('reverse').onclick=()=>{reversed=!reversed;draw()};draw();
