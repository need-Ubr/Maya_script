import pymel.core as pm
    
def splineIKsetting(partname):    
    seljnt = pm.ls(sl=1, fl=1, type="joint")
    
    if not '%s_DistanceGrp' % partname in pm.ls():
        DistanceGrp = pm.group(name='%s_DistanceGrp' % partname, em=1, world=1)
    
    else:
        DistanceGrp = '%s_DistanceGrp' % partname
    
    if not '%s_ScaleCrvGrp' % partname in pm.ls():
        ScaleCrvGrp = pm.group(name='%s_ScaleCrvGrp' % partname, em=1, world=1)
    
    else:
        ScaleCrvGrp = '%s_ScaleCrvGrp' % partname
    
    
    
    ikHdl, ikEft, ikCrv = pm.ikHandle(startJoint=seljnt[0], endEffector=seljnt[-1], name='%s_IK_Hdl'%partname, createCurve=1, solver='ikSplineSolver', simplifyCurve=0)
    ikEft.rename('%s_IK_Eft'%partname)
    ikCrv.rename('%s_IK_Crv'%partname)
    ikCrvShp = ikCrv.getShape()
    
    baseLength = 0
    jntLength =[0]
    for _jnt in seljnt[1:]:
        transX = pm.getAttr(_jnt.tx)
        baseLength += transX
        jntLength.append(transX)
    
    pointOnCrvInfo = []
    sumLength = 0
    for num in range(0, len(seljnt)):
        POCnode = pm.createNode("pointOnCurveInfo", name="%s_pointOnCurveInfo%s"%(partname, str(1 + num)))
        pointOnCrvInfo.append(POCnode)
    
        sumLength += jntLength[num]
    
        POCnode.parameter.set(sumLength/baseLength)
        POCnode.turnOnPercentage.set(1)
        ikCrv.worldSpace >> POCnode.inputCurve
        print("%s.worldSpace >> %s.inputCurve" % (ikCrv, POCnode))
    
    
    pointLoc = []
    for num in range(0,len(pointOnCrvInfo)):
        loc = pm.spaceLocator(name="%s_distanceLoc%s"%(partname, str(1 + num)))
        pm.parent(loc, DistanceGrp)
        pointLoc.append(loc)
        pointOnCrvInfo[num].result.position >> loc.t
        print("%s.result.position >> %s.translate" % (pointOnCrvInfo[num], loc))
    
    
    #scaleCrv part
    scaleCrv = []
    scaleCrvInfo = []
    for num in range(0, len(pointLoc)-1):
        vectorSpace1 = pm.getAttr(pointLoc[num].worldPosition)
        vectorSpace2 = pm.getAttr(pointLoc[num+1].worldPosition)
    
        Crv = pm.curve(degree=1, point=[(vectorSpace1[0], vectorSpace1[1], vectorSpace1[2]), (vectorSpace2[0], vectorSpace2[1], vectorSpace2[2])], name="%s_scaleCrv%s" % (partname, num+1))
        pm.parent(Crv, ScaleCrvGrp)
    
        CrvInfo = pm.shadingNode("curveInfo", asUtility=1, name="%s_curveInfo%s" % (partname, num + 1))
        Crv.getShape().worldSpace >> CrvInfo.inputCurve
    
        scaleCrv.append(Crv)
        scaleCrvInfo.append(CrvInfo)
    
    #distance part
    distance = []
    baseLength_MD =[]
    for num in range(0,len(pointOnCrvInfo)-1):
        dist_shp = pm.distanceDimension(sp=[1000,1000,1000], ep=[2005,2005,2005])
        dist = dist_shp.listRelatives(parent=1)[0]
        dist.rename("%s_distance%s"%(partname, str(1 + num)))
        distance.append(dist)
        pm.parent(dist, DistanceGrp)
    
        sploc = dist_shp.startPoint.listConnections()
        eploc = dist_shp.endPoint.listConnections()
    
        pointLoc[num].worldPosition >> dist_shp.startPoint
        pointLoc[num + 1].worldPosition >> dist_shp.endPoint
    
        print("%s.worldPosition >> %s.startPoint" % (pointLoc[num], dist_shp))
        print("%s.worldPosition >> %s.endPoint" % (pointLoc[num + 1], dist_shp))
    
        distValue = pm.getAttr(dist_shp.distance)
        pm.delete(sploc)
        pm.delete(eploc)
    
        Length_MD = pm.shadingNode("multiplyDivide", asUtility=1, name="%s_multiplyDivide%s"%(partname, num+1))
    
        dist_shp.distance >> Length_MD.input1.input1X
        scaleCrvInfo[num].arcLength >> Length_MD.input2.input2X
        print("%s.distance >> %s.input1.input1X" % (dist_shp, Length_MD))
        print("%s.arcLength >> %s.input2.input2X" % (scaleCrvInfo[num], Length_MD))
    
        Length_MD.operation.set(2)
        # Length_MD.input2.input2X.set(distValue)
    
        baseLength_MD.append(Length_MD)
    
    
    for jnt, md, in zip(seljnt[:-1], baseLength_MD):
        md.output.outputX >> jnt.sx
        print("%s.output.outputX >> %s.scaleX" % (md, jnt))
    
    
splineIKsetting('spine')
