"""Dependency-light browser editor for crawler_mujoco arena YAML files."""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
import threading
import tempfile
import webbrowser
import zlib
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import yaml

from .resources import package_share_directory


HTML = r'''<!doctype html><html lang="ja"><head><meta charset="utf-8">
<title>Crawler MuJoCo Environment Editor</title><style>
*{box-sizing:border-box}body{margin:0;background:#171a1f;color:#e9edf2;font:14px sans-serif;overflow:hidden}
button,input,select{background:#292f38;color:#eef;border:1px solid #566170;border-radius:4px;padding:6px}
button:disabled{opacity:.4;cursor:default}
textarea{width:100%;min-height:220px;resize:vertical;background:#11161c;color:#cfe8cf;border:1px solid #566170;border-radius:4px;padding:8px;font:12px/1.4 monospace;tab-size:2}
button{cursor:pointer}button:hover{background:#394454}.toolbar{height:52px;padding:8px;display:flex;gap:8px;align-items:center;border-bottom:1px solid #39414b}
.main{height:calc(100vh - 52px);display:grid;grid-template-columns:260px 1fr 310px}.panel{padding:12px;overflow:auto;border-right:1px solid #39414b}.right{border-left:1px solid #39414b;border-right:0}
.views{display:grid;grid-template-columns:1fr 1fr;grid-template-rows:58% 42%;min-width:0}.view{position:relative;min-height:0;border:1px solid #39414b}.render{grid-column:1/3}
canvas,#mujoco{width:100%;height:100%;display:block;background:#20252c;object-fit:contain}.label{position:absolute;left:8px;top:7px;background:#111a;padding:4px 7px;border-radius:4px;pointer-events:none}
.object{padding:8px;border-radius:5px;margin:4px 0;background:#252b33;cursor:pointer}.object.sel{background:#315676}.row{display:grid;grid-template-columns:55px 1fr;gap:6px;align-items:center;margin:7px 0}
.triple{display:grid;grid-template-columns:repeat(3,1fr);gap:4px}.triple input{width:100%;min-width:0}.group{margin:14px 0;padding-top:8px;border-top:1px solid #39414b}
#status{color:#9ed3a6;white-space:pre-wrap}.danger{background:#692f35}.primary{background:#315f91}h3{margin:4px 0 10px}small{color:#aeb7c4}
</style></head><body>
<div class="toolbar"><strong>MuJoCo Environment Editor</strong><button id="undo" title="元に戻す (Ctrl+Z)" disabled>↶ 戻す</button><button id="redo" title="やり直す (Ctrl+Y)" disabled>↷ 進む</button><select id="arena" title="パッケージ内のarena YAML"></select><button id="load">一覧から読込</button>
<input id="openFile" type="file" accept=".yaml,.yml,application/yaml,text/yaml" hidden><button id="open">YAMLを開く</button>
<input id="filename" value="custom_environment" title="別名保存するファイル名"><button class="primary" id="save">別名で保存</button><button id="overwrite" title="現在の読込元へ保存" disabled>読込元へ上書き</button><span id="status"></span></div>
<div class="main"><div class="panel"><h3>モデル追加</h3><select id="model" style="width:100%"></select><button id="add" style="width:100%;margin-top:7px">選択モデルを追加</button>
<div class="group"><h3>オブジェクト</h3><div id="objects"></div></div>
<small>左ドラッグ: 移動<br>右ドラッグ: 回転（上面Yaw／側面Pitch）<br>ホイール: ズーム<br>Shift+移動: 10 cm刻み</small></div>
<div class="views" id="views"><div class="view render"><img id="mujoco"><span class="label">MuJoCo 3D（ドラッグ: 視点回転／ホイール: ズーム）</span><input id="renderSize" type="range" min="30" max="80" value="58" title="3D表示領域の高さ" style="position:absolute;right:8px;top:7px;width:110px"></div><div class="view"><canvas id="top"></canvas><span class="label">上面 X/Y</span></div><div class="view"><canvas id="side"></canvas><span class="label">側面 X/Z</span></div></div>
<div class="panel right"><h3>選択オブジェクト</h3><div id="none">左の一覧またはビューから選択してください</div><div id="editor" hidden>
<div class="row"><label>名前</label><input id="name"></div><div class="row"><label>モデル</label><input id="modelName" disabled></div>
<div class="group">位置 [m]<div class="triple"><input id="x" type="number" step="0.05"><input id="y" type="number" step="0.05"><input id="z" type="number" step="0.05"></div></div>
<div class="group">回転 R/P/Y [deg]<div class="triple"><input id="roll" type="number" step="5"><input id="pitch" type="number" step="5"><input id="yaw" type="number" step="5"></div></div>
<div class="group">モデルローカル・スケール X/Y/Z<div class="triple"><input id="sx" type="number" min="0.01" step="0.05"><input id="sy" type="number" min="0.01" step="0.05"><input id="sz" type="number" min="0.01" step="0.05"></div></div>
<div class="group"><button id="duplicate">複製</button><button class="danger" id="remove">削除</button></div>
</div><div class="group"><h3>arena YAML</h3><textarea id="yaml" spellcheck="false" aria-label="arena YAML"></textarea><button id="applyYaml" style="width:100%;margin-top:7px">YAMLテキストを画面へ反映</button><small>画面上の編集はこのテキストへ自動反映されます。</small></div></div></div>
<script>
let models=[],objects=[],selected=-1,zoom=70,pan={x:0,y:0},drag=null,renderTimer=null,renderUrl=null,renderSerial=0;
let edits=[JSON.stringify([])],editIndex=0,camera={azimuth:135,elevation:-28,distance:0},cameraDrag=null,sourceArena='';
const $=s=>document.querySelector(s), fields=['x','y','z','roll','pitch','yaw','sx','sy','sz'];
async function api(url,opt){let r=await fetch(url,opt);let j=await r.json();if(!r.ok)throw Error(j.error||r.statusText);return j}
function setArenas(arenas,selected=''){$('#arena').innerHTML=arenas.map(a=>`<option${a===selected?' selected':''}></option>`).join('');[...$('#arena').options].forEach((option,i)=>{option.value=arenas[i];option.textContent=arenas[i]})}
function modelInfo(name){return models.find(m=>m.name===name)||{size:[1,1,1]}}
function uniqueName(model){let n=1;while(objects.some(o=>o.name===`${model}_${n}`))n++;return `${model}_${n}`}
function normalize(o){return {name:o.name,model:o.model,x:+o.x||0,y:+o.y||0,z:+o.z||0,roll:+o.roll||0,pitch:+o.pitch||0,yaw:+o.yaw||0,sx:+o.sx||1,sy:+o.sy||1,sz:+o.sz||1}}
function yamlNumber(value){let n=Number(value);return Number.isFinite(n)?String(n):'0'}
function yamlText(){let lines=['robocup_arena:','  objects:'];if(!objects.length)lines.push('    {}');objects.forEach(o=>{lines.push(`    ${o.name}:`);for(let f of ['x','y','z','roll','pitch','yaw'])lines.push(`      ${f}: ${yamlNumber(o[f])}`);for(let a of ['x','y','z'])lines.push(`      scale_${a}: ${yamlNumber(o['s'+a])}`)});return lines.join('\n')+'\n'}
function syncYaml(){if(document.activeElement!==$('#yaml'))$('#yaml').value=yamlText()}
function setSource(name){sourceArena=name||'';$('#overwrite').disabled=!sourceArena;$('#overwrite').title=sourceArena?`上書き先: ${sourceArena}`:'読込元がありません'}
function historyButtons(){$('#undo').disabled=editIndex<=0;$('#redo').disabled=editIndex>=edits.length-1}
function resetHistory(){edits=[JSON.stringify(objects)];editIndex=0;historyButtons()}
function commit(){let state=JSON.stringify(objects);if(state===edits[editIndex])return;edits=edits.slice(0,editIndex+1);edits.push(state);editIndex++;historyButtons()}
function restore(index){if(index<0||index>=edits.length)return;editIndex=index;objects=JSON.parse(edits[index]).map(normalize);selected=Math.min(selected,objects.length-1);select(selected);historyButtons()}
function addObject(model,x=0,y=0){objects.push(normalize({name:uniqueName(model),model,x,y,z:0,sx:1,sy:1,sz:1}));commit();select(objects.length-1)}
function select(i){selected=i;renderList();syncEditor();syncYaml();draw();scheduleRender()}
function renderList(){$('#objects').innerHTML=objects.map((o,i)=>`<div class="object ${i===selected?'sel':''}" data-i="${i}">${o.name}<br><small>${o.model} (${o.x.toFixed(2)}, ${o.y.toFixed(2)}, ${o.z.toFixed(2)})</small></div>`).join('');document.querySelectorAll('.object').forEach(e=>e.onclick=()=>select(+e.dataset.i))}
function syncEditor(){let ok=selected>=0&&objects[selected];$('#none').hidden=ok;$('#editor').hidden=!ok;if(!ok)return;let o=objects[selected];$('#name').value=o.name;$('#modelName').value=o.model;fields.forEach(f=>$('#'+f).value=o[f])}
function update(){if(selected<0)return;let o=objects[selected];o.name=$('#name').value||o.name;fields.forEach(f=>o[f]=+$('#'+f).value);renderList();syncYaml();draw();scheduleRender()}
function resize(c){let r=c.getBoundingClientRect(),d=devicePixelRatio||1;if(c.width!==r.width*d||c.height!==r.height*d){c.width=r.width*d;c.height=r.height*d}let x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);return [x,r.width,r.height]}
function worldToScreen(v,w,h,side){return [w/2+(v.x-pan.x)*zoom,h/2-(side?v.z:v.y-(side?0:pan.y))*zoom]}
function screenToWorld(px,py,w,h,side){return side?{x:(px-w/2)/zoom+pan.x,z:(h/2-py)/zoom}:{x:(px-w/2)/zoom+pan.x,y:(h/2-py)/zoom+pan.y}}
function grid(ctx,w,h,side){ctx.clearRect(0,0,w,h);ctx.lineWidth=1;let step=zoom<35?1:.5;for(let q=-20;q<=20;q+=step){let a=worldToScreen({x:q,y:-20,z:-20},w,h,side),b=worldToScreen({x:q,y:20,z:20},w,h,side);ctx.strokeStyle=Math.abs(q)<1e-8?'#8290a0':'#313842';ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();let c=side?worldToScreen({x:-20,z:q},w,h,true):worldToScreen({x:-20,y:q},w,h,false),d=side?worldToScreen({x:20,z:q},w,h,true):worldToScreen({x:20,y:q},w,h,false);ctx.strokeStyle=Math.abs(q)<1e-8?'#8290a0':'#313842';ctx.beginPath();ctx.moveTo(c[0],c[1]);ctx.lineTo(d[0],d[1]);ctx.stroke()}}
function corners(o,side){let s=modelInfo(o.model).size,dx=s[0]*o.sx/2,dy=(side?s[2]*o.sz:s[1]*o.sy)/2,a=(side?o.pitch:o.yaw)*Math.PI/180,c=Math.cos(a),d=Math.sin(a),cy=side?o.z:o.y;return [[-dx,-dy],[dx,-dy],[dx,dy],[-dx,dy]].map(p=>({x:o.x+p[0]*c-p[1]*d,y:cy+p[0]*d+p[1]*c}))}
function drawView(c,side){let [ctx,w,h]=resize(c);grid(ctx,w,h,side);objects.forEach((o,i)=>{let cs=corners(o,side).map(p=>worldToScreen({x:p.x,y:p.y,z:p.y},w,h,side));ctx.beginPath();cs.forEach((p,j)=>j?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();ctx.fillStyle=i===selected?'#43a5e688':'#d7954588';ctx.strokeStyle=i===selected?'#8fd4ff':'#ffc271';ctx.lineWidth=i===selected?3:1.5;ctx.fill();ctx.stroke();let p=worldToScreen({x:o.x,y:o.y,z:o.z},w,h,side);ctx.fillStyle='#fff';ctx.fillText(o.name,p[0]+5,p[1]-5)})}
function draw(){drawView($('#top'),false);drawView($('#side'),true)}
function scheduleRender(){clearTimeout(renderTimer);renderTimer=setTimeout(renderMujoco,180)}
async function renderMujoco(){let serial=++renderSerial;try{let r=await fetch('/api/render',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({objects,camera})});if(!r.ok)throw Error((await r.json()).error);let blob=await r.blob();if(serial!==renderSerial)return;let url=URL.createObjectURL(blob);$('#mujoco').src=url;if(renderUrl)URL.revokeObjectURL(renderUrl);renderUrl=url}catch(e){$('#status').textContent='3D render: '+e.message}}
function hit(canvas,e,side){let r=canvas.getBoundingClientRect(),p=screenToWorld(e.clientX-r.left,e.clientY-r.top,r.width,r.height,side);for(let i=objects.length-1;i>=0;i--){let o=objects[i],s=modelInfo(o.model).size;if(Math.abs(p.x-o.x)<=s[0]*o.sx*.7&&Math.abs((side?p.z-o.z:p.y-o.y))<=(side?s[2]*o.sz:s[1]*o.sy)*.7)return i}return -1}
function bindCanvas(c,side){c.oncontextmenu=e=>e.preventDefault();c.onpointerdown=e=>{let i=hit(c,e,side);if(i>=0)select(i);if(selected<0)return;let r=c.getBoundingClientRect(),p=screenToWorld(e.clientX-r.left,e.clientY-r.top,r.width,r.height,side),o=objects[selected];drag={dx:o.x-p.x,dy:(side?o.z-p.z:o.y-p.y),side,rotate:e.button===2,startX:e.clientX,startAngle:side?o.pitch:o.yaw};c.setPointerCapture(e.pointerId)};c.onpointermove=e=>{if(!drag)return;let o=objects[selected];if(drag.rotate){let angle=Math.round((drag.startAngle+(e.clientX-drag.startX)*.5)/(e.shiftKey?15:1))*(e.shiftKey?15:1);if(side)o.pitch=angle;else o.yaw=angle}else{let r=c.getBoundingClientRect(),p=screenToWorld(e.clientX-r.left,e.clientY-r.top,r.width,r.height,side),snap=e.shiftKey?.1:.01;o.x=Math.round((p.x+drag.dx)/snap)*snap;if(side)o.z=Math.max(0,Math.round((p.z+drag.dy)/snap)*snap);else o.y=Math.round((p.y+drag.dy)/snap)*snap}syncEditor();renderList();draw();scheduleRender()};c.onpointerup=()=>{if(drag)commit();drag=null;scheduleRender()};c.onwheel=e=>{e.preventDefault();zoom=Math.max(20,Math.min(250,zoom*(e.deltaY>0?.9:1.1)));draw()}}
function bindCamera(){let view=$('#mujoco');view.ondragstart=()=>false;view.onpointerdown=e=>{cameraDrag={x:e.clientX,y:e.clientY,azimuth:camera.azimuth,elevation:camera.elevation};view.setPointerCapture(e.pointerId)};view.onpointermove=e=>{if(!cameraDrag)return;camera.azimuth=cameraDrag.azimuth+(e.clientX-cameraDrag.x)*.45;camera.elevation=Math.max(-89,Math.min(10,cameraDrag.elevation-(e.clientY-cameraDrag.y)*.35));scheduleRender()};view.onpointerup=()=>cameraDrag=null;view.onwheel=e=>{e.preventDefault();let current=camera.distance||5;camera.distance=Math.max(.5,Math.min(100,current*(e.deltaY>0?1.12:.89)));scheduleRender()}}
async function init(){models=await api('/api/models');$('#model').innerHTML=models.map(m=>`<option value="${m.name}">${m.name} (${m.size.map(v=>v.toFixed(2)).join('×')} m)</option>`).join('');setArenas(await api('/api/arenas'));bindCanvas($('#top'),false);bindCanvas($('#side'),true);bindCamera();resetHistory();syncYaml();draw();scheduleRender()}
$('#undo').onclick=()=>restore(editIndex-1);$('#redo').onclick=()=>restore(editIndex+1);
$('#add').onclick=()=>addObject($('#model').value);$('#remove').onclick=()=>{if(selected<0)return;objects.splice(selected,1);commit();select(Math.min(selected,objects.length-1))};$('#duplicate').onclick=()=>{if(selected<0)return;let o={...objects[selected],name:uniqueName(objects[selected].model),x:objects[selected].x+.3};objects.push(o);commit();select(objects.length-1)};$('#name').onchange=()=>{update();commit()};fields.forEach(f=>{$('#'+f).oninput=update;$('#'+f).onchange=commit});
$('#load').onclick=async()=>{try{let arena=$('#arena').value,j=await api('/api/arena?name='+encodeURIComponent(arena));objects=j.objects.map(normalize);$('#filename').value=j.stem;setSource(arena);resetHistory();select(objects.length?0:-1);$('#status').textContent='読込: '+j.path}catch(e){$('#status').textContent=e.message}};
$('#open').onclick=()=>$('#openFile').click();
$('#openFile').onchange=async e=>{let file=e.target.files[0];if(!file)return;try{let j=await api('/api/open',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({filename:file.name,content:await file.text()})});objects=j.objects.map(normalize);$('#filename').value=j.stem;setSource('');resetHistory();select(objects.length?0:-1);$('#status').textContent='読込: '+file.name+'（別名保存してください）'}catch(error){$('#status').textContent=error.message}finally{e.target.value=''}};
$('#applyYaml').onclick=async()=>{try{let j=await api('/api/open',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({filename:$('#filename').value+'.yaml',content:$('#yaml').value})});objects=j.objects.map(normalize);commit();select(objects.length?0:-1);$('#status').textContent='YAMLテキストを画面へ反映しました'}catch(e){$('#status').textContent='YAMLエラー: '+e.message}};
async function save(target=''){let body={filename:$('#filename').value,objects};if(target)body.target=target;let j=await api('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});setSource(j.arena);setArenas(await api('/api/arenas'),j.arena);$('#status').textContent=(target?'上書き保存: ':'別名保存: ')+j.path+'（画面のYAMLと同じ内容です）'}
$('#save').onclick=async()=>{try{await save()}catch(e){$('#status').textContent=e.message}};
$('#overwrite').onclick=async()=>{if(!sourceArena)return;try{await save(sourceArena)}catch(e){$('#status').textContent=e.message}};
$('#renderSize').oninput=e=>{$('#views').style.gridTemplateRows=`${e.target.value}% ${100-e.target.value}%`;draw()};
document.onkeydown=e=>{if(!(e.ctrlKey||e.metaKey))return;if(e.key.toLowerCase()==='z'){e.preventDefault();restore(editIndex+(e.shiftKey?1:-1))}else if(e.key.toLowerCase()==='y'){e.preventDefault();restore(editIndex+1)}};
window.onresize=draw;init();
</script></body></html>'''


def _stl_bounds(path: Path) -> tuple[list[float], list[float]]:
    data = path.read_bytes()
    vertices: list[tuple[float, float, float]] = []
    if len(data) >= 84 and 84 + struct.unpack_from("<I", data, 80)[0] * 50 == len(data):
        count = struct.unpack_from("<I", data, 80)[0]
        for triangle in range(count):
            offset = 84 + triangle * 50 + 12
            vertices.extend(struct.unpack_from("<fff", data, offset + vertex * 12) for vertex in range(3))
    else:
        text = data.decode("utf-8", errors="ignore")
        vertices = [tuple(map(float, match)) for match in re.findall(
            r"vertex\s+([-+\deE.]+)\s+([-+\deE.]+)\s+([-+\deE.]+)", text)]
    if not vertices:
        return [0.0] * 3, [1.0] * 3
    return ([min(p[i] for p in vertices) for i in range(3)],
            [max(p[i] for p in vertices) for i in range(3)])


def model_catalog(model_root: Path) -> list[dict]:
    result = []
    for sdf_path in sorted(model_root.glob("*/model.sdf")):
        try:
            model = ET.parse(sdf_path).getroot().find("model")
            mesh = model.find(".//collision/geometry/mesh") if model is not None else None
            size = [1.0, 1.0, 1.0]
            if mesh is not None:
                uri = (mesh.findtext("uri") or "").strip()
                scale = [float(v) for v in (mesh.findtext("scale") or "1 1 1").split()]
                relative = uri[len("model://"):] if uri.startswith("model://") else uri
                mesh_path = model_root / relative if uri.startswith("model://") else sdf_path.parent / relative
                lower, upper = _stl_bounds(mesh_path)
                raw = [(upper[i] - lower[i]) * scale[i] for i in range(3)]
                pose = [float(v) for v in (model.findtext("pose") or "0 0 0 0 0 0").split()]
                # Current bundled obstacle correction is an X-axis quarter turn.
                size = [raw[0], raw[2], raw[1]] if abs(abs(pose[3]) - math.pi / 2) < 1e-3 else raw
            result.append({"name": sdf_path.parent.name, "size": size})
        except Exception:
            result.append({"name": sdf_path.parent.name, "size": [1.0, 1.0, 1.0]})
    return result


def arena_payload_from_config(config, *, path: str, stem: str) -> dict:
    if not isinstance(config, dict):
        raise ValueError("YAMLのルートはマッピングである必要があります")
    arena = config.get("robocup_arena", {})
    if not isinstance(arena, dict) or not isinstance(arena.get("objects", {}), dict):
        raise ValueError("robocup_arena.objects が見つかりません")
    objects = []
    for name, values in arena.get("objects", {}).items():
        match = re.match(r"^(.+)_([0-9]+)$", str(name))
        if not match or not isinstance(values, dict):
            continue
        item = {"name": str(name), "model": match.group(1)}
        for field in ("x", "y", "z", "roll", "pitch", "yaw"):
            item[field] = float(values.get(field, 0.0))
        for axis in "xyz":
            item[f"s{axis}"] = float(values.get(f"scale_{axis}", 1.0))
        objects.append(item)
    return {"path": path, "stem": stem, "objects": objects}


def arena_payload(path: Path) -> dict:
    config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return arena_payload_from_config(config, path=str(path), stem=path.stem)


def objects_to_arena(raw_objects: list, model_root: Path) -> dict:
    arena = {"robocup_arena": {"objects": {}}}
    used = set()
    for raw in raw_objects:
        model = str(raw.get("model", ""))
        if not (model_root / model / "model.sdf").is_file():
            raise ValueError(f"unknown model: {model}")
        name = re.sub(r"[^A-Za-z0-9_-]", "_", str(raw.get("name", "")))
        if not re.match(r"^.+_[0-9]+$", name) or name in used or not name.startswith(model + "_"):
            suffix = 1
            while f"{model}_{suffix}" in used:
                suffix += 1
            name = f"{model}_{suffix}"
        used.add(name)
        values = {field: float(raw.get(field, 0.0)) for field in
                  ("x", "y", "z", "roll", "pitch", "yaw")}
        for axis in "xyz":
            values[f"scale_{axis}"] = max(0.01, float(raw.get(f"s{axis}", 1.0)))
        arena["robocup_arena"]["objects"][name] = values
    return arena


def rgb_png(rgb) -> bytes:
    """Encode a uint8 RGB array as PNG using only the standard library."""
    height, width, channels = rgb.shape
    if channels != 3:
        raise ValueError("renderer image must be RGB")
    raw = b"".join(b"\0" + rgb[row].tobytes() for row in range(height))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + kind + data +
                struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff))

    return (b"\x89PNG\r\n\x1a\n" +
            chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) +
            chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def render_arena(raw_objects: list, model_root: Path, width=640, height=360,
                 camera_options: dict | None = None) -> bytes:
    import mujoco
    from .configuration import load_config
    from .simulator import build_xml

    arena = objects_to_arena(raw_objects, model_root)
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", encoding="utf-8") as stream:
        yaml.safe_dump(arena, stream, sort_keys=False, allow_unicode=True)
        stream.flush()
        config = load_config(package_share_directory() / "config" / "default.yaml")
        config["environment"]["arena_yaml"] = stream.name
        config["environment"]["spawn_robot"] = False
        model = mujoco.MjModel.from_xml_string(build_xml(config, "semicircle"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    if raw_objects:
        xs = [float(obj.get("x", 0.0)) for obj in raw_objects]
        ys = [float(obj.get("y", 0.0)) for obj in raw_objects]
        zs = [float(obj.get("z", 0.0)) for obj in raw_objects]
        camera.lookat[:] = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2,
                            max(0.4, (min(zs) + max(zs)) / 2 + 0.4))
        span = max(max(xs) - min(xs), max(ys) - min(ys), 2.0)
        camera.distance = max(4.0, span * 1.7)
    else:
        camera.lookat[:] = (0.0, 0.0, 0.4)
        camera.distance = 5.0
    camera_options = camera_options if isinstance(camera_options, dict) else {}
    camera.azimuth = float(camera_options.get("azimuth", 135.0))
    camera.elevation = max(-89.0, min(10.0, float(camera_options.get("elevation", -28.0))))
    requested_distance = float(camera_options.get("distance", 0.0))
    if requested_distance > 0:
        camera.distance = max(0.5, min(100.0, requested_distance))
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera=camera)
        return rgb_png(renderer.render())
    finally:
        renderer.close()


def run_editor(host: str = "127.0.0.1", port: int = 8766, open_browser: bool = True) -> None:
    root = package_share_directory()
    arena_root, model_root = root / "assets" / "arenas", root / "assets" / "models"
    save_root = arena_root / "custom"
    save_root.mkdir(parents=True, exist_ok=True)
    render_lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def reply(self, status: int, payload, content_type="application/json"):
            body = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
            self.send_response(status); self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

        def do_GET(self):
            parsed = urlparse(self.path)
            try:
                if parsed.path == "/api/models":
                    return self.reply(200, model_catalog(model_root))
                if parsed.path == "/api/arenas":
                    names = [str(path.relative_to(arena_root)) for path in sorted(arena_root.rglob("*.yaml"))]
                    return self.reply(200, names)
                if parsed.path == "/api/arena":
                    name = parse_qs(parsed.query).get("name", [""])[0]
                    path = (arena_root / name).resolve()
                    if arena_root.resolve() not in path.parents or not path.is_file():
                        raise ValueError("arenaが見つかりません")
                    return self.reply(200, arena_payload(path))
                self.reply(200, HTML.encode(), "text/html; charset=utf-8")
            except Exception as error:
                self.reply(400, {"error": str(error)})

        def do_POST(self):
            try:
                endpoint = urlparse(self.path).path
                data = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))) or b"{}")
                if endpoint == "/api/open":
                    content = str(data.get("content", ""))
                    if len(content.encode("utf-8")) > 5 * 1024 * 1024:
                        raise ValueError("YAMLファイルが大きすぎます (上限5 MB)")
                    filename = Path(str(data.get("filename", "environment.yaml"))).name
                    config = yaml.safe_load(content) or {}
                    stem = Path(filename).stem or "environment"
                    return self.reply(200, arena_payload_from_config(
                        config, path=filename, stem=stem))
                if endpoint == "/api/render":
                    with render_lock:
                        image = render_arena(data.get("objects", []), model_root,
                                             camera_options=data.get("camera"))
                    return self.reply(200, image, "image/png")
                if endpoint != "/api/save":
                    return self.reply(404, {"error": "not found"})
                target = str(data.get("target", ""))
                if target:
                    output = (arena_root / target).resolve()
                    if arena_root.resolve() not in output.parents or output.suffix not in {".yaml", ".yml"}:
                        raise ValueError("上書き先が不正です")
                else:
                    filename = re.sub(r"[^A-Za-z0-9_.-]", "_", str(data.get("filename", "environment")))
                    filename = filename.removesuffix(".yaml").removesuffix(".yml") + ".yaml"
                    output = save_root / filename
                arena = objects_to_arena(data.get("objects", []), model_root)
                output.write_text(yaml.safe_dump(arena, sort_keys=False, allow_unicode=True), encoding="utf-8")
                self.reply(200, {"path": str(output),
                                 "arena": str(output.relative_to(arena_root))})
            except Exception as error:
                self.reply(400, {"error": str(error)})

    server = ThreadingHTTPServer((host, port), Handler)
    url = f"http://{host}:{port}"
    print(f"Crawler MuJoCo environment editor: {url}")
    print(f"Custom arenas: {save_root}")
    if open_browser:
        threading.Timer(0.2, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Crawler MuJoCo browser environment editor")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    run_editor(args.host, args.port, not args.no_browser)


if __name__ == "__main__":
    main()
