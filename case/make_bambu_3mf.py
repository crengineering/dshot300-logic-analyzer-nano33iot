"""Builds la_case_bambu.3mf: a Bambu Studio project with
  - object "Lid"  = two parts, Lid (filament 1) + Label letters (filament 2),
    laid top-face-down on the plate,
  - object "Base" beside it, opening up.
Input: la_case_Lid.stl, la_case_Label.stl, la_case_Base.stl (Fusion exports, mm).
Run: python make_bambu_3mf.py [template.3mf]  — the optional template supplies
project_settings.config (printer/filament/process presets); without it Bambu
Studio uses the presets that are active when the project is opened.
"""
import struct, sys, os, uuid, zipfile, datetime

HERE = os.path.dirname(os.path.abspath(__file__))
VERSION = '02.08.02.61'
PLATE_POS = {'Lid': (120.0, 140.0), 'Base': (190.0, 140.0)}


def read_stl(path):
    with open(path, 'rb') as h:
        h.read(80); n = struct.unpack('<I', h.read(4))[0]
        data = h.read(50 * n)
    verts, idx, tris = [], {}, []
    for i in range(n):
        f = struct.unpack_from('<12f', data, 50 * i)
        t = []
        for k in range(3):
            v = (round(f[3 + 3 * k], 5), round(f[4 + 3 * k], 5), round(f[5 + 3 * k], 5))
            if v not in idx:
                idx[v] = len(verts); verts.append(v)
            t.append(idx[v])
        tris.append(tuple(t))
    return verts, tris


def transform(verts, flip, dx, dy, dz):
    out = []
    for x, y, z in verts:
        if flip:
            y, z = -y, -z              # 180 deg about X: top face goes to the bed
        out.append((x + dx, y + dy, z + dz))
    return out


def bounds(vs):
    xs, ys, zs = zip(*vs)
    return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)


def mesh_xml(oid, verts, tris):
    s = ['  <object id="%d" p:UUID="%s" type="model">\n   <mesh>\n    <vertices>\n' % (oid, uuid.uuid4())]
    s += ['     <vertex x="%.5f" y="%.5f" z="%.5f"/>\n' % v for v in verts]
    s.append('    </vertices>\n    <triangles>\n')
    s += ['     <triangle v1="%d" v2="%d" v3="%d"/>\n' % t for t in tris]
    s.append('    </triangles>\n   </mesh>\n  </object>\n')
    return ''.join(s)


HEAD = ('<?xml version="1.0" encoding="UTF-8"?>\n'
        '<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
        'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
        'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p">\n'
        ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n')


def main():
    template = sys.argv[1] if len(sys.argv) > 1 else None
    lid_v, lid_t = read_stl(os.path.join(HERE, 'la_case_Lid.stl'))
    lab_v, lab_t = read_stl(os.path.join(HERE, 'la_case_Label.stl'))
    base_v, base_t = read_stl(os.path.join(HERE, 'la_case_Base.stl'))

    # lid + label share one transform so the letters stay in their pockets
    flipped = transform(lid_v, True, 0, 0, 0)
    x0, x1, y0, y1, z0, z1 = bounds(flipped)
    d = (-(x0 + x1) / 2, -(y0 + y1) / 2, -z0)
    lid_v = transform(lid_v, True, *d); lab_v = transform(lab_v, True, *d)
    x0, x1, y0, y1, z0, z1 = bounds(base_v)
    base_v = transform(base_v, False, -(x0 + x1) / 2, -(y0 + y1) / 2, -z0)

    today = datetime.date.today().isoformat()
    # Bambu Studio keys parts by a document-wide unique id: (part index << 16) | object id
    # (bbs_3mf.cpp, volumes_objectID). Duplicate ids across sub-model files silently
    # replace geometry, so never reuse 1, 2, ... per file.
    main_ids = {'object_3.model': 3, 'object_4.model': 4}
    names = {'object_3.model': 'Lid', 'object_4.model': 'Base'}
    pid = lambda obj, idx: (idx << 16) | obj
    objs = {  # sub-model file -> [(part id, name, verts, tris, extruder)]
        'object_3.model': [(pid(3, 1), 'Lid', lid_v, lid_t, 1), (pid(3, 2), 'Label', lab_v, lab_t, 2)],
        'object_4.model': [(pid(4, 1), 'Base', base_v, base_t, 1)],
    }

    files = {}
    res, build, rels, cfg_objs, instances, assemble = [], [], [], [], [], []
    for i, (fname, parts) in enumerate(objs.items(), 1):
        files['3D/Objects/' + fname] = HEAD + ' <resources>\n' + ''.join(mesh_xml(pid, v, t) for pid, _, v, t, _ in parts) + ' </resources>\n <build/>\n</model>\n'
        rels.append(' <Relationship Target="/3D/Objects/%s" Id="rel-%d" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n' % (fname, i))
        mid = main_ids[fname]
        comps = ''.join('    <component p:path="/3D/Objects/%s" objectid="%d" p:UUID="%s" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>\n' % (fname, pid, uuid.uuid4()) for pid, *_ in parts)
        res.append('  <object id="%d" p:UUID="%s" type="model">\n   <components>\n%s   </components>\n  </object>\n' % (mid, uuid.uuid4(), comps))
        px, py = PLATE_POS[names[fname]]
        build.append('  <item objectid="%d" p:UUID="%s" transform="1 0 0 0 1 0 0 0 1 %g %g 0" printable="1"/>\n' % (mid, uuid.uuid4(), px, py))
        part_xml = ''
        for pid, pname, v, t, ext in parts:
            part_xml += ('    <part id="%d" subtype="normal_part" uuid="%s">\n      <metadata key="name" value="%s"/>\n'
                         '      <metadata key="extruder" value="%d"/>\n      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>\n'
                         '      <mesh_stat face_count="%d" edges_fixed="0" degenerate_facets="0" facets_removed="0" facets_reversed="0" backwards_edges="0"/>\n    </part>\n'
                         % (pid, uuid.uuid4(), pname, ext, len(t)))
        cfg_objs.append('  <object id="%d">\n    <metadata key="name" value="%s"/>\n    <metadata key="extruder" value="1"/>\n    <metadata face_count="%d"/>\n%s  </object>\n'
                        % (mid, names[fname], sum(len(t) for *_, t, _ in parts), part_xml))
        instances.append('    <model_instance>\n      <metadata key="object_id" value="%d"/>\n      <metadata key="instance_id" value="0"/>\n      <metadata key="identify_id" value="%d"/>\n    </model_instance>\n' % (mid, 100 + mid))
        assemble.append('   <assemble_item object_id="%d" instance_id="0" transform="1 0 0 0 1 0 0 0 1 %g %g 0" offset="0 0 0" />\n' % (mid, px, py))

    files['3D/3dmodel.model'] = (HEAD.replace(' <metadata name="BambuStudio:3mfVersion">1</metadata>\n',
                                 ' <metadata name="Application">BambuStudio-%s</metadata>\n <metadata name="BambuStudio:3mfVersion">1</metadata>\n'
                                 ' <metadata name="CreationDate">%s</metadata>\n <metadata name="ModificationDate">%s</metadata>\n <metadata name="Title">Logic Analyzer Case</metadata>\n' % (VERSION, today, today))
                                 + ' <resources>\n' + ''.join(res) + ' </resources>\n <build p:UUID="%s">\n' % uuid.uuid4() + ''.join(build) + ' </build>\n</model>\n')
    files['3D/_rels/3dmodel.model.rels'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
                                            + ''.join(rels) + '</Relationships>\n')
    files['Metadata/model_settings.config'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>\n' + ''.join(cfg_objs)
        + '  <plate>\n    <metadata key="plater_id" value="1"/>\n    <metadata key="plater_name" value=""/>\n    <metadata key="locked" value="false"/>\n'
          '    <metadata key="filament_map_mode" value="Auto For Flush"/>\n    <metadata key="filament_maps" value="1 1 1"/>\n'
          '    <metadata key="filament_volume_maps" value="0 0 0"/>\n    <metadata key="thumbnail_file" value="Metadata/plate_1.png"/>\n'
        + ''.join(instances) + '  </plate>\n  <assemble>\n' + ''.join(assemble) + '  </assemble>\n</config>\n')
    files['Metadata/slice_info.config'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>\n  <header>\n    <header_item key="X-BBL-Client-Type" value="slicer"/>\n'
                                           '    <header_item key="X-BBL-Client-Version" value="%s"/>\n  </header>\n</config>\n' % VERSION)
    files['Metadata/cut_information.xml'] = '<?xml version="1.0" encoding="utf-8"?>\n<objects>\n' + ''.join(' <object id="%d">\n  <cut_id id="0" check_sum="1" connectors_cnt="0"/>\n </object>\n' % m for m in main_ids.values()) + '</objects>\n'
    files['Metadata/filament_sequence.json'] = '{"plate_1":{"nozzle_sequence":[],"optimal_assignment":[],"sequence":[]}}'
    files['[Content_Types].xml'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">\n'
                                    ' <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>\n'
                                    ' <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>\n'
                                    ' <Default Extension="png" ContentType="image/png"/>\n</Types>\n')
    files['_rels/.rels'] = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
                            ' <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n'
                            ' <Relationship Target="/Metadata/plate_1.png" Id="rel-2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail"/>\n'
                            '</Relationships>\n')
    if template:
        with zipfile.ZipFile(template) as z:
            files['Metadata/project_settings.config'] = z.read('Metadata/project_settings.config')
    prev = os.path.join(HERE, 'la_case_preview.png')
    if os.path.exists(prev):
        files['Metadata/plate_1.png'] = open(prev, 'rb').read()

    out = os.path.join(HERE, 'la_case_bambu.3mf')
    with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as z:
        for n, d in files.items():
            z.writestr(n, d)
    print('wrote', out, '- lid %d tris, label %d tris, base %d tris' % (len(lid_t), len(lab_t), len(base_t)))


if __name__ == '__main__':
    main()
