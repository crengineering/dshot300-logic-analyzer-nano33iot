import adsk.core, adsk.fusion

# ---- parameters (mm) -------------------------------------------------------
P = dict(
    # Arduino Nano 33 IoT. Measured board: 45.5 x 18 mm (the datasheet drawing
    # ABX00027 sec. 6 says 43.16 x 17.77 - a base printed to that size was too
    # short for the real board). 4 mounting holes d1.65 on the standard Nano
    # 40.64 x 15.24 mm grid, assumed centred (2.43 mm from the ends); micro-USB
    # overhangs the +X board end by ~1.6 mm; header rows 15.24 mm apart.
    pcbL=45.5, pcbW=18.0, pcbT=1.6,
    holeX=20.32, holeY=7.62, holeD=1.65,
    clr=0.5,                             # pocket clearance around the PCB, per side
    wall=1.6, floorT=1.6,
    standoff=3.5,                        # PCB underside above the floor (untrimmed header tails are ~3 mm)
    headroom=5.5,                        # lid underside above PCB top (USB overmold 7 mm centred at +1.4)
    lidT=1.6, skirtT=1.6, skirtH=4.0, lidClear=0.2,
    nubH=0.6, nubL=8.0, nubT=1.2, nubX=14.0, nubZ=1.5, nubClr=0.15,  # snap ridges on the long walls: proud, length, height, +-x centre, z above skirt bottom, groove clearance
    cornerR=2.0,
    ledgeD=3.8,                          # end ledges under the PCB (must carry the locating pins)
    pinD=1.3, pinH=1.4,                  # locating pins on the ledges, into the 4 PCB holes (hole d1.65, PCB 1.6 thick)
    bossW=2.2, bossIn=0.7, bossGap=0.3,  # lid bosses over the holes: width (y), reach inboard of the hole centre (x), gap above the PCB top
    rowPitch=15.24, slotW=3.2, slotL=41.0,
    usbW=13.0, usbOver=1.6,              # notch width (notch runs from the floor level to the rim); connector overhang beyond the PCB end
    usbReliefW=11.0,                     # the +X ledge is cut away this wide under the USB socket (its shell tabs stick out below the PCB)
    textH=4.5, textGap=1.0, textZ=0.6, font='Arial',
    pinMarkH=3.0, pinMarkGap=0.4, gndDotD=1.8,   # pin marks beside the header slots: font size, gap to the slot, GND dot diameter
    pinMarks=('T', '4', '3', '2', '1'),  # pins 1..5 from the USB end on the D12..D8 row (-Y): TLM, M4, M3, M2, M1
    gndPins=((-1, 12), (1, 14)),         # (row sign, pin number from the USB end) of the two GND pins
    textTop='LOGIC', textBot='ANALYZER',
)
def c(mm): return mm / 10.0            # API works in cm
VI = adsk.core.ValueInput
P3 = adsk.core.Point3D.create

def rect(sk, x0, y0, x1, y1):
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(c(x0), c(y0), 0), P3(c(x1), c(y1), 0))

def all_profiles(sk):
    col = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count): col.add(sk.profiles.item(i))
    return col

def extrude(comp, prof, z0, dist, op, participants=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(prof, op)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(VI.createByReal(c(z0)))
    inp.setDistanceExtent(False, VI.createByReal(c(dist)))
    if participants: inp.participantBodies = participants
    return ext.add(inp)

def vertical_edges(body, pred=lambda x, y: True):
    col = adsk.core.ObjectCollection.create()
    for i in range(body.edges.count):
        e = body.edges.item(i)
        s, t = e.startVertex.geometry, e.endVertex.geometry
        if abs(s.x - t.x) < 1e-6 and abs(s.y - t.y) < 1e-6 and abs(s.z - t.z) > 1e-6 and pred(s.x * 10, s.y * 10):
            col.add(e)
    return col

def fillet(comp, edges, r):
    f = comp.features.filletFeatures
    inp = f.createInput()
    inp.edgeSetInputs.addConstantRadiusEdgeSet(edges, VI.createByReal(c(r)), True)
    f.add(inp)

def chamfer(comp, edges, d):
    ch = comp.features.chamferFeatures
    inp = ch.createInput(edges, True)
    inp.setToEqualDistance(VI.createByReal(c(d)))
    ch.add(inp)

def x_edges(body, pred):
    col = adsk.core.ObjectCollection.create()
    for i in range(body.edges.count):
        e = body.edges.item(i)
        s, t = e.startVertex.geometry, e.endVertex.geometry
        if abs(s.y - t.y) < 1e-6 and abs(s.z - t.z) < 1e-6 and abs(s.x - t.x) > 1e-6 and pred(s.y * 10, s.z * 10):
            col.add(e)
    return col

def run(_context):
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    for k, v in P.items():
        if isinstance(v, float): design.userParameters.add(k, VI.createByReal(c(v)), 'mm', 'LA case')

    Lo = P['pcbL'] + 2 * (P['clr'] + P['wall']); Wo = P['pcbW'] + 2 * (P['clr'] + P['wall'])
    Lc = P['pcbL'] + 2 * P['clr'];           Wc = P['pcbW'] + 2 * P['clr']
    Hb = P['floorT'] + P['standoff'] + P['pcbT'] + P['headroom']
    zPcbTop = P['floorT'] + P['standoff'] + P['pcbT']
    Ll = Lo + 2 * (P['lidClear'] + P['skirtT']); Wl = Wo + 2 * (P['lidClear'] + P['skirtT'])
    Li = Lo + 2 * P['lidClear'];                 Wi = Wo + 2 * P['lidClear']
    ox = P['clr'] + P['wall'] - P['usbOver']    # board offset in +X: USB connector face flush with the case end
    NEW, JOIN, CUT = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                      adsk.fusion.FeatureOperations.JoinFeatureOperation,
                      adsk.fusion.FeatureOperations.CutFeatureOperation)

    # ---- Base ---------------------------------------------------------------
    baseOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    base = baseOcc.component
    sk = base.sketches.add(base.xYConstructionPlane); rect(sk, -Lo/2, -Wo/2, Lo/2, Wo/2)
    body = extrude(base, sk.profiles.item(0), 0, Hb, NEW).bodies.item(0); body.name = 'Base'
    fillet(base, vertical_edges(body), P['cornerR'])
    sk = base.sketches.add(base.xYConstructionPlane); rect(sk, -Lc/2 + ox, -Wc/2, Lc/2 + ox, Wc/2)
    extrude(base, sk.profiles.item(0), P['floorT'], Hb - P['floorT'], CUT, [body])
    sk = base.sketches.add(base.xYConstructionPlane)
    rect(sk, -Lc/2 + ox, -Wc/2, -Lc/2 + ox + P['ledgeD'], Wc/2); rect(sk, Lc/2 + ox - P['ledgeD'], -Wc/2, Lc/2 + ox, Wc/2)
    extrude(base, all_profiles(sk), 0, P['floorT'] + P['standoff'], JOIN, [body])
    sk = base.sketches.add(base.xYConstructionPlane)                  # locating pins into the PCB holes
    for sx in (-1, 1):
        for sy in (-1, 1):
            sk.sketchCurves.sketchCircles.addByCenterRadius(P3(c(sx * P['holeX'] + ox), c(sy * P['holeY']), 0), c(P['pinD'] / 2))
    extrude(base, all_profiles(sk), P['floorT'] + P['standoff'], P['pinH'], JOIN, [body])
    sk = base.sketches.add(base.xYConstructionPlane)                  # relief in the +X ledge under the USB socket, down to the floor
    rect(sk, Lc/2 + ox - P['ledgeD'] - 0.01, -P['usbReliefW']/2, Lc/2 + ox, P['usbReliefW']/2)
    extrude(base, sk.profiles.item(0), P['floorT'], Hb - P['floorT'], CUT, [body])
    sk = base.sketches.add(base.xYConstructionPlane)
    rect(sk, Lc/2 + ox, -P['usbW']/2, Lo/2 + 1, P['usbW']/2)         # USB opening through the +X wall
    extrude(base, sk.profiles.item(0), P['floorT'], Hb, CUT, [body])
    zNub = Hb - P['skirtH'] + P['nubZ']                                # snap ridges, 2 per long wall
    sk = base.sketches.add(base.xYConstructionPlane)
    for sx in (-1, 1):
        for sy in (-1, 1):
            x0, x1 = sorted([sx * P['nubX'] - P['nubL']/2, sx * P['nubX'] + P['nubL']/2])
            y0, y1 = sorted([sy * Wo/2, sy * (Wo/2 + P['nubH'])])
            rect(sk, x0, y0, x1, y1)
    extrude(base, all_profiles(sk), zNub, P['nubT'], JOIN, [body])
    chamfer(base, x_edges(body, lambda y, z: abs(y) > Wo/2 + P['nubH'] - 0.01), P['nubH'] * 0.8)

    # ---- Lid ----------------------------------------------------------------
    lidOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    lid = lidOcc.component
    sk = lid.sketches.add(lid.xYConstructionPlane); rect(sk, -Ll/2, -Wl/2, Lo/2, Wl/2)   # plate ends flush with the base wall at the USB end
    lbody = extrude(lid, sk.profiles.item(0), Hb, P['lidT'], NEW).bodies.item(0); lbody.name = 'Lid'
    sk = lid.sketches.add(lid.xYConstructionPlane)                    # U-shaped skirt: long sides + the -X end, open at the USB end
    rect(sk, -Li/2, Wi/2, Lo/2, Wl/2); rect(sk, -Li/2, -Wl/2, Lo/2, -Wi/2); rect(sk, -Ll/2, -Wl/2, -Li/2, Wl/2)
    extrude(lid, all_profiles(sk), Hb - P['skirtH'], P['skirtH'], JOIN, [lbody])
    xin = Li/2 + P['skirtT']/2
    fillet(lid, vertical_edges(lbody, lambda x, y: x < -xin), P['cornerR'] + P['lidClear'] + P['skirtT'])
    fillet(lid, vertical_edges(lbody, lambda x, y: -xin < x < 0), P['cornerR'] + P['lidClear'])
    fillet(lid, vertical_edges(lbody, lambda x, y: x > 0 and abs(y) > Wi/2 + 0.01), P['cornerR'])
    sk = lid.sketches.add(lid.xYConstructionPlane)                    # grooves for the snap ridges
    for sx in (-1, 1):
        for sy in (-1, 1):
            x0, x1 = sorted([sx * P['nubX'] - P['nubL']/2 - P['nubClr'] * 2, sx * P['nubX'] + P['nubL']/2 + P['nubClr'] * 2])
            y0, y1 = sorted([sy * (Wi/2 - 0.05), sy * (Wi/2 + P['nubH'] + P['nubClr'])])
            rect(sk, x0, y0, x1, y1)
    extrude(lid, all_profiles(sk), zNub - P['nubClr'], P['nubT'] + 2 * P['nubClr'], CUT, [lbody])
    sk = lid.sketches.add(lid.xYConstructionPlane)                    # header slots
    for sy in (-1, 1):
        rect(sk, -P['slotL']/2 + ox, sy * P['rowPitch']/2 - P['slotW']/2, P['slotL']/2 + ox, sy * P['rowPitch']/2 + P['slotW']/2)
    extrude(lid, all_profiles(sk), Hb - 0.5, P['lidT'] + 1, CUT, [lbody])
    sk = lid.sketches.add(lid.xYConstructionPlane)                    # 4 hold-down bosses: hole centre to the PCB end
    for sx in (-1, 1):                                                # (run through the lid: they overlap the slot ends;
        for sy in (-1, 1):                                            #  slot stays free from -19.6 to +19.6, headers need 38.1)
            x0, x1 = sorted([sx * (P['holeX'] - P['bossIn']) + ox, sx * P['pcbL']/2 + ox])
            y0, y1 = sorted([sy * (P['holeY'] - P['bossW']/2), sy * (P['holeY'] + P['bossW']/2)])
            rect(sk, x0, y0, x1, y1)
    extrude(lid, all_profiles(sk), zPcbTop + P['bossGap'], Hb + P['lidT'] - zPcbTop - P['bossGap'], JOIN, [lbody])

    # ---- Label (engraved, separate body for a second filament) ------------------
    zTop = Hb + P['lidT']
    pinp = lid.constructionPlanes.createInput()
    pinp.setByOffset(lid.xYConstructionPlane, VI.createByReal(c(zTop)))
    plane = lid.constructionPlanes.add(pinp)
    sk = lid.sketches.add(plane)
    h, g = P['textH'], P['textGap']
    boxL = P['slotL']/2 - 1.0
    tools = adsk.core.ObjectCollection.create()
    for txt, yc in ((P['textTop'], (h + g)/2), (P['textBot'], -(h + g)/2)):
        ti = sk.sketchTexts.createInput2(txt, c(h))
        ti.setAsMultiLine(P3(c(ox - boxL), c(yc - h/2), 0), P3(c(ox + boxL), c(yc + h/2), 0),
                          adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                          adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
        ti.fontName = P['font']
        ti.textStyle = adsk.fusion.TextStyles.TextStyleBold
        st = sk.sketchTexts.add(ti)
        ef = lid.features.extrudeFeatures.addSimple(st, VI.createByReal(-c(P['textZ'])), NEW)
        for i in range(ef.bodies.count):
            b = ef.bodies.item(i); b.name = 'Label'; tools.add(b)
    # pin marks: one character per pin beside the -Y slot (header pins sit one pitch inside the mounting holes)
    pitch = 2.54
    ym = P['rowPitch']/2 + P['slotW']/2 + P['pinMarkGap'] + P['pinMarkH']/2
    hm = P['pinMarkH']
    for k, txt in enumerate(P['pinMarks'], start=1):
        xp = ox + P['holeX'] - pitch * k
        ti = sk.sketchTexts.createInput2(txt, c(hm))
        ti.setAsMultiLine(P3(c(xp - pitch/2), c(-ym - hm/2), 0), P3(c(xp + pitch/2), c(-ym + hm/2), 0),
                          adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                          adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
        ti.fontName = P['font']
        ti.textStyle = adsk.fusion.TextStyles.TextStyleBold
        st = sk.sketchTexts.add(ti)
        ef = lid.features.extrudeFeatures.addSimple(st, VI.createByReal(-c(P['textZ'])), NEW)
        for i in range(ef.bodies.count):
            b = ef.bodies.item(i); b.name = 'Label'; tools.add(b)
    skd = lid.sketches.add(plane)                                     # GND dots
    for sy, k in P['gndPins']:
        skd.sketchCurves.sketchCircles.addByCenterRadius(P3(c(ox + P['holeX'] - pitch * k), c(sy * ym), 0), c(P['gndDotD']/2))
    ef = extrude(lid, all_profiles(skd), zTop - P['textZ'], P['textZ'], NEW)
    for i in range(ef.bodies.count):
        b = ef.bodies.item(i); b.name = 'Label'; tools.add(b)
    ci = lid.features.combineFeatures.createInput(lbody, tools)
    ci.operation = CUT; ci.isKeepToolBodies = True
    lid.features.combineFeatures.add(ci)

    app.activeViewport.fit()
    print('OK base %.1f x %.1f x %.1f mm, lid %.1f x %.1f, label bodies %d, params %d' %
          (Lo, Wo, Hb, Ll, Wl, tools.count, design.userParameters.count))
