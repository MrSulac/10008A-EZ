/* Pure data utilities shared by the offline viewer and its tests. */
const AutonData = (() => {
  const angleDelta = (a,b) => ((b-a+180)%360+360)%360-180;
  function normalize(input, options={}) {
    if (!Array.isArray(input) || !input.length || input.length>100000) throw Error('Expected 1–100,000 position samples.');
    const scale={inches:1,meters:1/.0254,millimeters:1/25.4}[options.units||'inches'];
    if (!scale) throw Error('Unsupported distance unit.');
    const output=[];
    for (const raw of input) {
      const t=Number(raw.t ?? raw.time ?? raw.time_s), x=Number(raw.x)*scale, y=Number(raw.y)*scale;
      const value=raw.theta ?? raw.heading;
      const theta=value===undefined || value==='' ? null : Number(value)*(options.headingUnit==='radians'?180/Math.PI:1);
      if (![t,x,y].every(Number.isFinite)||t<0||t>86400||Math.abs(x)>1e6||Math.abs(y)>1e6||(theta!==null&&!Number.isFinite(theta))) throw Error('Each sample needs finite t, x, y and an optional heading.');
      if (output.length && t<output.at(-1).t) throw Error('Timestamps must increase. Import each match phase separately.');
      const point={t,x,y,theta,v:raw.v===undefined?null:Number(raw.v)*scale};
      if(point.v!==null&&!Number.isFinite(point.v)) throw Error('Invalid velocity.');
      if(output.length&&t===output.at(-1).t) output[output.length-1]=point;else output.push(point);
    }
    for(let i=0;i<output.length;i++) {
      const p=output[i], a=output[Math.max(0,i-1)], b=output[Math.min(output.length-1,i+1)];
      if(p.theta===null) p.theta=Math.hypot(b.x-a.x,b.y-a.y)>1e-8?Math.atan2(b.x-a.x,b.y-a.y)*180/Math.PI:(i?output[i-1].theta:0);
      if(p.v===null) p.v=b.t>a.t?Math.hypot(b.x-a.x,b.y-a.y)/(b.t-a.t):0;
    }
    return output;
  }
  function rows(text) {
    const result=[];let row=[],field='',quoted=false;
    for(let i=0;i<text.length;i++) {
      const c=text[i];
      if(c==='"') {if(quoted&&text[i+1]==='"'){field+='"';i++;}else quoted=!quoted;}
      else if(c===','&&!quoted){row.push(field);field='';}
      else if((c==='\n'||c==='\r')&&!quoted){if(c==='\r'&&text[i+1]==='\n')i++;row.push(field);if(row.some(x=>x.trim()))result.push(row);row=[];field='';}
      else field+=c;
    }
    if(quoted)throw Error('Unclosed CSV quote.');
    row.push(field);if(row.some(x=>x.trim()))result.push(row);
    return result;
  }
  function parse(text, options={}) {
    if(text.length>20*1024*1024)throw Error('Choose a file smaller than 20 MB.');
    if(text.trim().startsWith('{')||text.trim().startsWith('[')) {
      const data=JSON.parse(text), samples=Array.isArray(data)?data:data.samples;
      return {name:data.name||'Imported path',samples:normalize(samples,{units:data.units||options.units,headingUnit:data.headingUnit||options.headingUnit})};
    }
    const all=rows(text.replace(/^\uFEFF/,'')),meta={};
    const data=all.filter(row=>{if(row[0].startsWith('#')){meta[row[0].slice(1)]=row.slice(1).join(',');return false;}return true;});
    if(data.length<2)throw Error('CSV needs column headers and at least one sample.');
    const header=data.shift().map(s=>s.trim().toLowerCase());
    const index=(...names)=>header.findIndex(x=>names.includes(x));
    const ti=index('t','time','time_s','time_ms'),xi=index('x'),yi=index('y'),hi=index('theta','heading','heading_deg'),vi=index('v','velocity','speed');
    if([ti,xi,yi].some(i=>i<0))throw Error('CSV columns must include t (seconds), x and y. theta (degrees) is optional.');
    const warnings=[];if(data.length&&!/[\r\n]$/.test(text)&&data.at(-1).length<header.length){data.pop();warnings.push('Ignored incomplete final CSV row.');}
    const samples=data.map(row=>({t:row[ti]?.trim()===''?NaN:Number(row[ti])/(header[ti]==='time_ms'?1000:1),x:row[xi]?.trim()===''?NaN:Number(row[xi]),y:row[yi]?.trim()===''?NaN:Number(row[yi]),theta:hi>=0?row[hi]:undefined,v:vi>=0?row[vi]:undefined}));
    return {name:meta.name||'Imported run',samples:normalize(samples,options),warnings};
  }
  function at(samples,t) {
    if(!samples.length)return null;if(t<=samples[0].t)return {...samples[0]};if(t>=samples.at(-1).t)return {...samples.at(-1)};
    let low=0,high=samples.length-1;while(high-low>1){const mid=(low+high)>>1;if(samples[mid].t<=t)low=mid;else high=mid;}
    const a=samples[low],b=samples[high],u=(t-a.t)/(b.t-a.t);
    return {t,x:a.x+(b.x-a.x)*u,y:a.y+(b.y-a.y)*u,theta:a.theta+angleDelta(a.theta,b.theta)*u,v:a.v+(b.v-a.v)*u};
  }
  const length = samples => samples.reduce((n,p,i)=>n+(i?Math.hypot(p.x-samples[i-1].x,p.y-samples[i-1].y):0),0);
  function deviation(samples,p) {
    if(!samples.length||!p)return null;let best=Infinity;
    for(let i=1;i<samples.length;i++){const a=samples[i-1],b=samples[i],dx=b.x-a.x,dy=b.y-a.y,d=dx*dx+dy*dy,u=d?Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/d)):0;best=Math.min(best,Math.hypot(p.x-a.x-u*dx,p.y-a.y-u*dy));}
    return Number.isFinite(best)?best:Math.hypot(p.x-samples[0].x,p.y-samples[0].y);
  }
  return {normalize,parse,at,length,deviation,angleDelta};
})();
if(typeof module!=='undefined')module.exports=AutonData;
