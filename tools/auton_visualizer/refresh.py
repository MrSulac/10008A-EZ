#!/usr/bin/env python3
"""Build an offline auton viewer from selector entries and real C++ path math.
Only literal/arithmetic command arguments are previewed. No auton is executed.
"""
import argparse
import ast
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

def clean(text):
    return re.sub(r'("(?:\\.|[^"\\])*")|//[^\n]*|/\*.*?\*/', lambda m: m.group(1) or ' ', text, flags=re.S)

def block(text, start, left='(', right=')'):
    depth = 0
    quoted = False
    escaped = False
    for i in range(start, len(text)):
        c = text[i]
        if quoted:
            if escaped: escaped = False
            elif c == '\\': escaped = True
            elif c == '"': quoted = False
            continue
        if c == '"': quoted = True; continue
        if c == left: depth += 1
        if c == right:
            depth -= 1
            if depth == 0: return text[start + 1:i], i + 1
    raise ValueError('Unbalanced source delimiters')

def split_args(text):
    result, begin, depth = [], 0, 0
    for i, c in enumerate(text):
        if c in '({[': depth += 1
        elif c in ')}]': depth -= 1
        elif c == ',' and depth == 0: result.append(text[begin:i].strip()); begin = i + 1
    result.append(text[begin:].strip())
    return result

def number(text, constants):
    text = re.sub(r'(?<=\d)_(?:in|deg|ms|s)\b', '', text.strip())
    def visit(node):
        if isinstance(node, ast.Constant) and type(node.value) in (int, float): return float(node.value)
        if isinstance(node, ast.Name) and node.id in constants: return constants[node.id]
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
            return visit(node.operand) * (-1 if isinstance(node.op, ast.USub) else 1)
        if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
            a, b = visit(node.left), visit(node.right)
            if isinstance(node.op, ast.Add): return a + b
            if isinstance(node.op, ast.Sub): return a - b
            if isinstance(node.op, ast.Mult): return a * b
            return a / b
        raise ValueError('Nonliteral expression: ' + text)
    value = visit(ast.parse(text, mode='eval').body)
    if not math.isfinite(value): raise ValueError('Nonfinite expression')
    return value

def functions(root):
    result = {}
    for path in sorted((root / 'src/autons').glob('*.cpp')):
        source = clean(path.read_text())
        for match in re.finditer(r'\bvoid\s+(\w+)\s*\([^)]*\)\s*\{', source):
            body, _ = block(source, match.end() - 1, '{', '}')
            result[match.group(1)] = (body, str(path.relative_to(root)))
    return result

def literals(text, constants):
    return [number(v, constants) for v in split_args(text)]

def points(text, constants):
    found = []
    for m in re.finditer(r'\{\s*([^{}]+)\}', text):
        args = split_args(m.group(1))
        if len(args) in (2, 3):
            try: found.append(literals(m.group(1), constants))
            except (ValueError, SyntaxError, ZeroDivisionError): pass
    return found

def compile_exporter(root, directory):
    compiler = shutil.which(os.environ.get('CXX', 'c++'))
    if not compiler: raise RuntimeError('A C++17 compiler is required to regenerate exact profile previews.')
    target = Path(directory) / 'export_profile'
    subprocess.run([compiler, '-std=c++17', '-O2', '-I'+str(root/'tests/ekf_profile/stubs'),
                    '-I'+str(root/'include'), str(root/'tools/auton_visualizer/export_profile.cpp'), '-o', str(target)], check=True)
    return target

def profile(body, root, constants, exporter):
    names = re.findall(r'\bmotion::Path\s+(\w+)', body)
    calls = [m for name in names for m in re.finditer(r'\b'+re.escape(name)+r'\.build\s*\(', body)]
    if not calls: return None
    if len(calls) != 1: raise ValueError('Multiple profile builds: import the recorded run for the full sequence.')
    call = calls[0]
    args, _ = block(body, call.end() - 1)
    waypoints = points(split_args(args)[0], constants)
    if len(waypoints) < 2 or any(len(p) != 3 for p in waypoints): raise ValueError('Profile uses nonliteral waypoints; import a recorded trace.')
    header = clean((root/'include/motion_profile.hpp').read_text())
    defaults = {}
    for name, expression in re.findall(r'double\s+(\w+)\s*=\s*([^;]+);', header):
        try: defaults[name] = number(expression, constants)
        except (SyntaxError, ValueError): pass
    config = clean((root/'include/ekf.hpp').read_text())
    defaults['trackWidth'] = number(re.search(r'trackWidth\s*=\s*([^;]+);', config).group(1), constants)
    for name, expression in re.findall(r'\blimits\.(\w+)\s*=\s*([^;]+);', body): defaults[name] = number(expression, constants)
    order = ['maxVelocity', 'maxAcceleration', 'maxWheelVelocity', 'maxWheelAcceleration', 'maxLateralAcceleration', 'maxAngularVelocity', 'trackWidth', 'sampleSpacing']
    data = str(len(waypoints))+'\n'+'\n'.join(' '.join(map(str,p)) for p in waypoints)+'\n'+' '.join(str(defaults[k]) for k in order)+'\n'
    out = subprocess.run([str(exporter)], input=data, text=True, capture_output=True, check=True)
    return json.loads(out.stdout), waypoints

class Geometry:
    def __init__(self, constants, width, max_speed):
        self.constants, self.width, self.max_speed = constants, width, max_speed
        self.samples = [dict(t=0., x=0., y=0., theta=0., v=0.)]
        self.events = []
    def move(self, x, y, theta, seconds, label):
        old = self.samples[-1]
        angle = math.remainder(theta-old['theta'], 360)
        steps = max(1, math.ceil(seconds/.03))
        speed = math.hypot(x-old['x'],y-old['y'])/max(seconds,.001)
        self.events.append(dict(t=old['t'],label=label))
        for i in range(1,steps+1):
            u=i/steps
            self.samples.append(dict(t=old['t']+seconds*u,x=old['x']+(x-old['x'])*u,y=old['y']+(y-old['y'])*u,theta=old['theta']+angle*u,v=speed))
    def drive(self, distance, speed, label):
        old=self.samples[-1]; a=math.radians(old['theta'])
        v=max(1,min(127,abs(speed)))/127*self.max_speed
        # Geometric/estimated timing only: simple triangular/trapezoidal timing.
        acceleration=30; d=abs(distance); ramp=v*v/acceleration
        seconds=2*math.sqrt(d/acceleration) if d<=ramp else 2*v/acceleration+(d-ramp)/v
        self.move(old['x']+distance*math.sin(a),old['y']+distance*math.cos(a),old['theta'],max(.03,seconds),label)
    def turn(self, target, speed, label):
        old=self.samples[-1]; seconds=abs(math.remainder(target-old['theta'],360))/max(1,min(180,abs(speed)/127*360))
        self.move(old['x'],old['y'],target,max(.03,seconds),label)
    def swing(self, side, target, speed, opposite):
        if abs(opposite)>=abs(speed): raise ValueError('Swing speed ratio cannot be previewed.')
        old=self.samples[-1]; a=math.radians(old['theta']); delta=math.radians(math.remainder(target-old['theta'],360))
        radius=(1 if side=='ez::LEFT_SWING' else -1)*self.width*.5*(abs(speed)+opposite)/(abs(speed)-opposite)
        seconds=max(.03,abs(math.degrees(delta))/90); steps=max(1,math.ceil(seconds/.03))
        self.events.append(dict(t=old['t'],label='Swing heading guide'))
        for i in range(1,steps+1):
            angle=a+delta*i/steps
            self.samples.append(dict(t=old['t']+seconds*i/steps,x=old['x']+radius*(math.cos(a)-math.cos(angle)),y=old['y']+radius*(math.sin(angle)-math.sin(a)),theta=math.degrees(angle),v=abs(radius*delta)/seconds))

def legacy(body, constants, width, max_speed):
    if re.search(r'\b(for|while|if)\s*\(', body):
        return [], [], ['Conditional or calibration routine: import its match log to see the actual branch.'], 'Recorded run needed'
    geometry=Geometry(constants,width,max_speed); notes=['EZ command geometry; PID dynamics, chaining and timing are approximations.']
    expression=r'\b(chassis\.(?:pid_drive_set|pid_odom_set|pid_turn_set|pid_turn_relative_set|pid_swing_set)|pros::delay|bot::\w+\.setTargetHeight)\s*\('
    for match in re.finditer(expression,body):
        args,_=block(body,match.end()-1); args=split_args(args); method=match.group(1)
        if method=='pros::delay':
            old=geometry.samples[-1];geometry.move(old['x'],old['y'],old['theta'],number(args[0],constants)/1000,'Wait')
        elif 'setTargetHeight' in method:
            geometry.events.append(dict(t=geometry.samples[-1]['t'],label='Lift target '+args[0]))
        elif method.endswith('pid_swing_set'):
            geometry.swing(args[0],number(args[1],constants),number(args[2],constants),number(args[3],constants) if len(args)>3 else 0)
            notes.append('Swing guide assumes wheel speed is proportional to motor command and uses the configured track width.')
        elif 'turn' in method:
            target=number(args[0],constants)+(geometry.samples[-1]['theta'] if 'relative' in method else 0)
            geometry.turn(target,number(args[1],constants),'Turn')
        elif method.endswith('pid_odom_set') and '{' in args[0]:
            targets=points(args[0],constants)
            if not targets: raise ValueError('Waypoint expression requires a recorded run.')
            notes.append('EZ pursuit/boomerang shows waypoint connections, not its generated pursuit curve.')
            reverse='ez::rev' in args[0]
            for p in targets:
                old=geometry.samples[-1]; angle=math.degrees(math.atan2(p[0]-old['x'],p[1]-old['y']))+(180 if reverse else 0)
                geometry.move(p[0],p[1],p[2] if len(p)==3 else angle,max(.1,math.hypot(p[0]-old['x'],p[1]-old['y'])/24),'Waypoint')
        else: geometry.drive(number(args[0],constants),number(args[1],constants),'Drive')
    if not geometry.events: notes.append('No recognized chassis motion; import a trace for custom code.')
    return geometry.samples,geometry.events,list(dict.fromkeys(notes)),'EZ geometry guide'

def build_catalog(root, exporter):
    main=clean((root/'src/main.cpp').read_text())
    selector=re.search(r'\.autons_add\s*\(',main)
    entries,_=block(main,selector.end()-1)
    registrations=re.findall(r'\{\s*"((?:\\.|[^"\\])*)"\s*,\s*(\w+)\s*\}',entries)
    funcs=functions(root); constants={}
    for name,value in re.findall(r'\b(?:const\s+)?int\s+(\w+)\s*=\s*(-?\d+)\s*;',clean((root/'include/autons/auton-config.hpp').read_text())): constants[name]=float(value)
    width=number(re.search(r'trackWidth\s*=\s*([^;]+);',clean((root/'include/ekf.hpp').read_text())).group(1),constants)
    constructor=re.search(r'ez::Drive\s+chassis\s*\(',main); drive_args,_=block(main,constructor.end()-1); drive_args=split_args(drive_args)
    max_speed=math.pi*number(drive_args[3],constants)*number(drive_args[4],constants)/60
    autons=[]
    for label,function in registrations:
        label=json.loads('"'+label+'"');title=label.split('\n')[0]
        body,filename=funcs.get(function,('', ''))
        row=dict(id=function,name=title,description=label.replace('\n',' ').strip(),source=filename,code=body.strip(),samples=[],events=[],waypoints=[],notes=[])
        try:
            exact=profile(body,root,constants,exporter)
            if exact:
                row['samples'],row['waypoints']=exact
                row['kind']='Exact profile reference'
                row['controller']='Ramsete' if 'ramsete::' in body else 'Stanley' if 'stanley::' in body else 'Default Ramsete'
                row['notes']=['Actual C++ path-generator output. This is the planned reference, not a prediction of physical tracking.']
            else:
                row['samples'],row['events'],row['notes'],row['kind']=legacy(body,constants,width,max_speed)
                row['controller']='EZ-Template'
        except (ValueError,SyntaxError,ZeroDivisionError,subprocess.CalledProcessError) as error:
            row['kind']='Recorded run needed';row['controller']='Custom';row['notes']=[str(error)]
        autons.append(row)
    walls=[]
    field=clean((root/'src/fieldmap.cpp').read_text())
    for m in re.finditer(r'\{\s*\{\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\}\s*,\s*\{\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\}',field): walls.append(list(map(float,m.groups())))
    return dict(version=1,units='inches',headingUnit='degrees',fieldSize=144,walls=walls,autons=autons)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=HERE/'index.html');args=parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='auton-profile-export-') as directory:
        catalog=build_catalog(ROOT,compile_exporter(ROOT,directory))
    payload=json.dumps(catalog,separators=(',',':'),allow_nan=False).replace('</','<\\/')
    html=(HERE/'viewer.template.html').read_text().replace('__AUTON_CATALOG__',payload).replace('__VIEWER_CORE__',(HERE/'viewer_core.js').read_text())
    args.output.write_text(html)
    print(f'Created {args.output}: {len(catalog["autons"])} registered autons; '+str(sum(a['kind']=='Exact profile reference' for a in catalog['autons']))+' exact profiles.')
if __name__=='__main__': main()
