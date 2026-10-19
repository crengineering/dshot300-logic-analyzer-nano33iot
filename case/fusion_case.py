import adsk.core, adsk.fusion

# ---- parameters (mm) -------------------------------------------------------
P = dict(
    pcbL=45.0, pcbW=18.0, pcbT=1.6,      # Arduino Nano 33 IoT
    clr=0.4,                             # pocket clearance around the PCB, per side
    wall=1.6, floorT=1.6,
    standoff=2.5,                        # PCB underside above the floor (solder stubs)
    headroom=5.5,                        # lid underside above PCB top (USB overmold 7 mm centred at +1.4)
    lidT=1.6, skirtT=1.6, skirtH=4.0, lidClear=0.2,
    nubH=0.6, nubL=8.0, nubT=1.2, nubX=14.0, nubZ=1.5, nubClr=0.15,  # snap ridges on the long walls: proud, length, height, +-x centre, z above skirt bottom, groove clearance
    cornerR=2.0,
    ledgeD=2.0,                          # end ledges under the PCB
    postX=1.6, postY=2.2,                # hold-down posts on the PCB corners
    rowPitch=15.24, slotW=3.2, slotL=41.0,
    usbW=12.0, usbDown=2.5,              # notch width; notch bottom below PCB top
    textH=4.5, textGap=1.0, textZ=0.6, font='Arial',
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
    NEW, JOIN, CUT = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                      adsk.fusion.FeatureOperations.JoinFeatureOperation,
                      adsk.fusion.FeatureOperations.CutFeatureOperation)

    # ---- Base ---------------------------------------------------------------
    baseOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    base = baseOcc.component
    sk = base.sketches.add(base.xYConstructionPlane); rect(sk, -Lo/2, -Wo/2, Lo/2, Wo/2)
    body = extrude(base, sk.profiles.item(0), 0, Hb, NEW).bodies.item(0); body.name = 'Base'
    fillet(base, vertical_edges(body), P['cornerR'])
    sk = base.sketches.add(base.xYConstructionPlane); rect(sk, -Lc/2, -Wc/2, Lc/2, Wc/2)
    extrude(base, sk.profiles.item(0), P['floorT'], Hb - P['floorT'], CUT, [body])
    sk = base.sketches.add(base.xYConstructionPlane)
    rect(sk, -Lc/2, -Wc/2, -Lc/2 + P['ledgeD'], Wc/2); rect(sk, Lc/2 - P['ledgeD'], -Wc/2, Lc/2, Wc/2)
    extrude(base, all_profiles(sk), 0, P['floorT'] + P['standoff'], JOIN, [body])
    sk = base.sketches.add(base.xYConstructionPlane)
    rect(sk, Lc/2, -P['usbW']/2, Lo/2 + 1, P['usbW']/2)              # USB notch, +X end
    extrude(base, sk.profiles.item(0), zPcbTop - P['usbDown'], Hb, CUT, [body])
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
    sk = lid.sketches.add(lid.xYConstructionPlane); rect(sk, -Ll/2, -Wl/2, Ll/2, Wl/2)
    lbody = extrude(lid, sk.profiles.item(0), Hb, P['lidT'], NEW).bodies.item(0); lbody.name = 'Lid'
    sk = lid.sketches.add(lid.xYConstructionPlane)
    rect(sk, -Ll/2, -Wl/2, Ll/2, Wl/2); rect(sk, -Li/2, -Wi/2, Li/2, Wi/2)
    ring = [sk.profiles.item(i) for i in range(sk.profiles.count) if sk.profiles.item(i).profileLoops.count == 2][0]
    extrude(lid, ring, Hb - P['skirtH'], P['skirtH'], JOIN, [lbody])
    xin = Li/2 + P['skirtT']/2
    fillet(lid, vertical_edges(lbody, lambda x, y: abs(x) > xin), P['cornerR'] + P['lidClear'] + P['skirtT'])
    fillet(lid, vertical_edges(lbody, lambda x, y: abs(x) < xin), P['cornerR'] + P['lidClear'])
    sk = lid.sketches.add(lid.xYConstructionPlane)                    # grooves for the snap ridges
    for sx in (-1, 1):
        for sy in (-1, 1):
            x0, x1 = sorted([sx * P['nubX'] - P['nubL']/2 - P['nubClr'] * 2, sx * P['nubX'] + P['nubL']/2 + P['nubClr'] * 2])
            y0, y1 = sorted([sy * (Wi/2 - 0.05), sy * (Wi/2 + P['nubH'] + P['nubClr'])])
            rect(sk, x0, y0, x1, y1)
    extrude(lid, all_profiles(sk), zNub - P['nubClr'], P['nubT'] + 2 * P['nubClr'], CUT, [lbody])
    sk = lid.sketches.add(lid.xYConstructionPlane)
    rect(sk, Lo/2 - 0.5, -P['usbW']/2, Ll/2 + 1, P['usbW']/2)        # skirt notch over the USB
    extrude(lid, sk.profiles.item(0), Hb - P['skirtH'], P['skirtH'], CUT, [lbody])
    sk = lid.sketches.add(lid.xYConstructionPlane)                    # 4 hold-down posts
    for sx in (-1, 1):
        for sy in (-1, 1):
            x0, x1 = sorted([sx * P['pcbL']/2, sx * (P['pcbL']/2 - P['postX'])])
            y0, y1 = sorted([sy * P['pcbW']/2, sy * (P['pcbW']/2 - P['postY'])])
            rect(sk, x0, y0, x1, y1)
    extrude(lid, all_profiles(sk), zPcbTop + 0.2, Hb - zPcbTop - 0.2 + 0.01, JOIN, [lbody])
    sk = lid.sketches.add(lid.xYConstructionPlane)                    # header slots
    for sy in (-1, 1):
        rect(sk, -P['slotL']/2, sy * P['rowPitch']/2 - P['slotW']/2, P['slotL']/2, sy * P['rowPitch']/2 + P['slotW']/2)
    extrude(lid, all_profiles(sk), Hb - 0.5, P['lidT'] + 1, CUT, [lbody])

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
        ti.setAsMultiLine(P3(c(-boxL), c(yc - h/2), 0), P3(c(boxL), c(yc + h/2), 0),
                          adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                          adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
        ti.fontName = P['font']
        ti.textStyle = adsk.fusion.TextStyles.TextStyleBold
        st = sk.sketchTexts.add(ti)
        ef = lid.features.extrudeFeatures.addSimple(st, VI.createByReal(-c(P['textZ'])), NEW)
        for i in range(ef.bodies.count):
            b = ef.bodies.item(i); b.name = 'Label'; tools.add(b)
    ci = lid.features.combineFeatures.createInput(lbody, tools)
    ci.operation = CUT; ci.isKeepToolBodies = True
    lid.features.combineFeatures.add(ci)

    app.activeViewport.fit()
    print('OK base %.1f x %.1f x %.1f mm, lid %.1f x %.1f, label bodies %d, params %d' %
          (Lo, Wo, Hb, Ll, Wl, tools.count, design.userParameters.count))
