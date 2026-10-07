"""将 UE 导出的 White Drone OBJ 拆成机身及四副独立旋翼，保留 UV。"""
import argparse
from collections import defaultdict
from pathlib import Path

PIVOTS = [(57.53, -62.65), (57.74, 62.48), (-60.28, 62.28), (-61.74, -62.28)]
CUT_HEIGHT = 15.0

def split(source, output):
    vertices, uvs, faces = [], [], []
    for line in source.read_text().splitlines():
        s = line.split()
        if not s:
            continue
        if s[0] == 'v':
            vertices.append(tuple(map(float, s[1:4])))
        elif s[0] == 'vt':
            uvs.append(tuple(map(float, s[1:3])))
        elif s[0] == 'f':
            faces.append([tuple(int(x)-1 for x in t.split('/')[:2]) for t in s[1:]])
    parent = list(range(len(vertices)))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    welded = {}
    for i, v in enumerate(vertices):
        key = tuple(round(x, 3) for x in v)
        if key in welded:
            parent[i] = welded[key]
        else:
            welded[key] = i
    upper = [f for f in faces if all(vertices[i][1] > CUT_HEIGHT for i, _ in f)]
    for f in upper:
        r = root(f[0][0])
        for i, _ in f[1:]:
            parent[root(i)] = r
    groups = defaultdict(list)
    for f in upper:
        groups[root(f[0][0])].extend(vertices[i] for i, _ in f)
    rotor_groups = {}
    for r, pts in groups.items():
        if max(p[0] for p in pts)-min(p[0] for p in pts) < 60 or min(p[0] for p in pts) < 0 < max(p[0] for p in pts):
            continue
        x = sum(p[0] for p in pts)/len(pts)
        y = sum(p[2] for p in pts)/len(pts)
        index = min(range(4), key=lambda j: (PIVOTS[j][0]-x)**2+(PIVOTS[j][1]-y)**2)
        rotor_groups[r] = index
    assert len(rotor_groups) == 4, 'Expected four separate rotors above the motor plane'
    meshes = [[] for _ in range(5)]
    def clip(poly, above):
        result = []
        for a, b in zip(poly, poly[1:]+poly[:1]):
            ia = (a[0][1] >= CUT_HEIGHT) if above else (a[0][1] <= CUT_HEIGHT)
            ib = (b[0][1] >= CUT_HEIGHT) if above else (b[0][1] <= CUT_HEIGHT)
            if ia:
                result.append(a)
            if ia != ib:
                t = (CUT_HEIGHT-a[0][1])/(b[0][1]-a[0][1])
                result.append(tuple(tuple(x+t*(y-x) for x,y in zip(aa,bb)) for aa,bb in zip(a,b)))
        return result
    def append(index, poly):
        for j in range(1,len(poly)-1):
            meshes[index].append([poly[0],poly[j],poly[j+1]])
    for f in faces:
        poly = [(vertices[i], uvs[t]) for i,t in f]
        rotor = next((rotor_groups[root(i)] for i,_ in f if root(i) in rotor_groups), None)
        if rotor is None:
            append(0, poly)
        else:
            append(0, clip(poly,False))
            append(rotor+1, clip(poly,True))
    output.mkdir(parents=True,exist_ok=True)
    for index, mesh in enumerate(meshes):
        pivot = (0,0) if index == 0 else PIVOTS[index-1]
        lines = ['# OBJ import coordinates: X, -Y, Z; rotor pivot at shaft centre', 'o UAV']
        for tri in mesh:
            for p,uv in tri:
                lines.append(f'v {p[0]-pivot[0]:.6f} {pivot[1]-p[2]:.6f} {p[1]:.6f}')
        for tri in mesh:
            for p,uv in tri:
                lines.append(f'vt {uv[0]:.6f} {uv[1]:.6f}')
        for i in range(len(mesh)):
            a = i*3+1
            lines.append('f '+' '.join(f'{j}/{j}' for j in range(a,a+3)))
        name = 'SM_WhiteDrone_Body' if index == 0 else f'SM_WhiteDrone_Rotor{index-1}'
        (output/(name+'.obj')).write_text('\n'.join(lines)+'\n')
        print(name, len(mesh), 'triangles')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    split(args.source, args.output)


