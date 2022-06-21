#include"occApp.h"
#include <IGESControl_Controller.hxx>
#include <IGESControl_Writer.hxx>
#include <TopLoc_Datum3D.hxx>

TopoDS_Shape OccApp::getShape()
{
	return myShape;
}

void OccApp::flush1()
{
	if (myShape.IsNull())
		myShape = shapeTmp;
	else
		myShape = BRepAlgoAPI_Fuse(myShape, shapeTmp);
}

void OccApp::flush2()
{
	TopoDS_Compound tc;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(tc);
	aBuilder.Add(tc, myShape);
	aBuilder.Add(tc, shapeTmp);
	myShape = tc;
}

TopoDS_Face* OccApp::makeFace()
{
	face = sketch.getFace();
	return (TopoDS_Face*)face;
}

TopoDS_Wire* OccApp::makeWire()
{
	face = sketch.getWire();
	return (TopoDS_Wire*)face;
}

/*
OccApp::OccApp():sketch(Sketch(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)))
{
	gp_Pnt p(0, 0, 0);
	ox = BRepBuilderAPI_MakeEdge(p, gp_Pnt(50, 0, 0));
	oy = BRepBuilderAPI_MakeEdge(p, gp_Pnt(0, 100, 0));
	oz = BRepBuilderAPI_MakeEdge(p, gp_Pnt(0, 0, 150));
	aBuilder.MakeCompound(CompoundShape);
}
*/

OccApp::OccApp()
{
}

TopoDS_Shape OccApp::makeBall(const gp_Pnt& center, const Standard_Real& radius)
{
	shapeTmp = BRepPrimAPI_MakeSphere(center, radius).Shape();
	return shapeTmp;
}

TopoDS_Shape OccApp::extrusion(gp_Vec dir1, const Standard_Real len1, gp_Vec dir2, const Standard_Real len2)
{
	if (len1 <= 0 || len2 <= 0 || face == NULL)
		return TopoDS_Shape();

	TopoDS_Shape ts1 = extrusion(dir1, len1);
	extrusion(dir2, len2);
	shapeTmp = BRepAlgoAPI_Fuse(shapeTmp, ts1);
	return shapeTmp;
}

TopoDS_Shape OccApp::extrusion(gp_Vec &vec, const Standard_Real& len)
{
	if (len <= 0 || !face)
		return TopoDS_Shape();

	vec.Normalize();
	vec *= len;
	shapeTmp = BRepPrimAPI_MakePrism(*face, vec);

	return shapeTmp;
}

TopoDS_Shape OccApp::extrusion(const Standard_Real& len1, const Standard_Real& len2)
{
	if (len1 <= 0 || len2 <= 0 || !face)
		return TopoDS_Shape();

	gp_Vec v = sketch.getNormal();
	TopoDS_Shape ts1 = extrusion(v, len1);
	v.Reverse();
	extrusion(v, len2);
	shapeTmp = BRepAlgoAPI_Fuse(shapeTmp, ts1);
	return shapeTmp;
}

TopoDS_Shape OccApp::extrusion(gp_Pnt& p1)
{
	gp_Vec v(gp_XYZ(p1.XYZ() - sketch.getLocation().XYZ()));
	Standard_Real r = v.Dot(sketch.getNormal());
	return extrusion(sketch.getNormal(), r);
}

TopoDS_Shape OccApp::extrusion(const Standard_Real& len)
{
	if (len <= 0 || !face)
		return TopoDS_Shape();
	gp_Vec v = sketch.getNormal();
	v *= len;
	shapeTmp = BRepPrimAPI_MakePrism(*face, v);
	return shapeTmp;
}

TopoDS_Shape OccApp::extrusionCut(gp_Vec& dir, const Standard_Real& len)
{
	if (len <= 0)
		return TopoDS_Shape();
	myShape = shapeTmp;
	extrusion(dir, len);
	myShape = BRepAlgoAPI_Cut(myShape, shapeTmp);
	return shapeTmp;
}

TopoDS_Shape OccApp::extrusionCut(gp_Vec& dir1, Standard_Real& len1, gp_Vec& dir2, Standard_Real& len2)
{
	extrusion(dir1, len1, dir2, len2);
	myShape = BRepAlgoAPI_Cut(myShape, shapeTmp);
	return myShape;
}

TopoDS_Shape OccApp::extrusionCut(const Standard_Real& len)
{
	return extrusionCut(sketch.getNormal(), len);
}

TopoDS_Shape OccApp::revolveExtrusion(const gp_Ax1& centerAxis, const Standard_Real& angle1, const Standard_Real& angle2)
{	
	TopoDS_Shape ts = BRepPrimAPI_MakeRevol(*face, centerAxis, angle1, false);
	if (angle2 > 0) {
		TopoDS_Shape ts1 = BRepPrimAPI_MakeRevol(*face, centerAxis, -angle2, false);
		ts = BRepAlgoAPI_Fuse(ts, ts1);
	}
	shapeTmp = ts;
	return shapeTmp;
}

TopoDS_Shape OccApp::revolveCut(const gp_Ax1& centerAxis, const Standard_Real& angle1, const Standard_Real& angle2)
{
	TopoDS_Shape ts = BRepPrimAPI_MakeRevol(*face, centerAxis, angle1, false);
	shapeTmp = BRepAlgoAPI_Cut(shapeTmp, ts);

	if (angle2 * angle1 < 0) {
		ts = BRepPrimAPI_MakeRevol(*face, centerAxis, angle1, false);
		shapeTmp = BRepAlgoAPI_Cut(shapeTmp, ts);
	}

	return shapeTmp;
}

TopoDS_Shape OccApp::linearPattern(gp_Vec& dir, const Standard_Real& gapDis, const Standard_Integer& num)
{
	gp_Trsf gt;
	dir.Normalize();
	gp_Vec tmp;
	TopoDS_Shape ts = shapeTmp, ts1;

	for (Standard_Integer i = 0; i < num-1; i++) {
		tmp = dir * (i + 1) * gapDis;
		gt.SetTranslationPart(tmp);
		BRepBuilderAPI_Transform bt(shapeTmp, gt);
		ts1 = bt.Shape();
		ts = BRepAlgoAPI_Fuse(ts, ts1);
	}

	shapeTmp = ts;
	return shapeTmp;
}

TopoDS_Shape OccApp::linearPattern(gp_Vec& dir, const Standard_Integer& num, const Standard_Real& distance)
{
	if(num <= 1)
		return TopoDS_Shape();
	else
	{
		linearPattern(dir, distance / (num - 1), num);
		return shapeTmp;
	}
}

TopoDS_Shape OccApp::facePattern(gp_Vec& dir1, const Standard_Real& gapDis1, const Standard_Integer& num1, 
	gp_Vec& dir2, const Standard_Real& gapDis2, const Standard_Integer& num2)
{
	linearPattern(dir1, gapDis1, num1);
	linearPattern(dir2, gapDis2, num2);
	return shapeTmp;
}

TopoDS_Shape OccApp::volumePattern(gp_Vec& dir1, const Standard_Real& gapDis1, const Standard_Integer& num1, 
	gp_Vec& dir2, const Standard_Real& gapDis2, const Standard_Integer& num2, 
	gp_Vec& dir3, const Standard_Real& gapDis3, const Standard_Integer& num3)
{
	facePattern(dir1, gapDis1, num1, dir2, gapDis1, num2);
	linearPattern(dir3, gapDis3, num3);
	return shapeTmp;
}

TopoDS_Shape OccApp::linearPatternCut(gp_Vec& dir, const Standard_Real& gapDis, const Standard_Integer& num)
{
	linearPattern(dir, gapDis, num);
	myShape = BRepAlgoAPI_Cut(myShape, shapeTmp);
	return myShape;
}

TopoDS_Shape OccApp::facePatternCut(gp_Vec& dir1, const Standard_Real& gapDis1, const Standard_Integer& num1, 
	gp_Vec& dir2, const Standard_Real& gapDis2, const Standard_Integer& num2)
{
	facePattern(dir1, gapDis1, num1, dir2, gapDis2, num2);
	myShape = BRepAlgoAPI_Cut(myShape, shapeTmp);
	return myShape;
}



TopoDS_Shape OccApp::circularPattern(const gp_Ax1& centerAx, const Standard_Real& gapRadian, const Standard_Integer& num)
{

	gp_Trsf gt;
	Standard_Real tmp = gapRadian;
	TopoDS_Shape ts = shapeTmp, ts1;

	for (Standard_Integer i = 0; i < num - 1; i++) {
		tmp =  (i + 1) * gapRadian;
		gt.SetRotation(centerAx, tmp);
		
		BRepBuilderAPI_Transform bt(shapeTmp, gt);
		ts1 = bt.Shape();
		ts = BRepAlgoAPI_Fuse(ts, ts1);
	}

	shapeTmp = ts;
	return shapeTmp;
}

TopoDS_Shape OccApp::circularPatternCut(const gp_Ax1& centerAx, const Standard_Real& gapRadian, const Standard_Integer& num)
{
	circularPattern(centerAx, gapRadian, num);
	myShape = BRepAlgoAPI_Cut(myShape, shapeTmp);
	return TopoDS_Shape();
}

TopoDS_Shape OccApp::offset()
{
	
	return TopoDS_Shape();
}

TopoDS_Shape OccApp::mirror(const gp_Ax1 &ax)
{
	gp_Trsf gt;
	gt.SetMirror(ax);
	BRepBuilderAPI_Transform bt(shapeTmp, gt, TRUE);
	TopoDS_Shape ts = bt.Shape();
	shapeTmp = BRepAlgoAPI_Fuse(shapeTmp, ts);
	return shapeTmp;
}

TopoDS_Shape OccApp::mirror(const gp_Ax2& ax) {
	gp_Trsf gt;
	gt.SetMirror(ax);
	BRepBuilderAPI_Transform bt(shapeTmp, gt, TRUE);
	TopoDS_Shape ts = bt.Shape();
	shapeTmp = BRepAlgoAPI_Fuse(shapeTmp, ts);
	return shapeTmp;
}

TopoDS_Shape OccApp::mirrorMyself(const gp_Ax2& ax)
{
	shapeTmp = myShape;
	mirror(ax);
	myShape = shapeTmp;
	return myShape;
}

Sketch* OccApp::createSketch(const gp_Ax2& coor)
{
	sketch.setCoordinate(coor);
	return &sketch;
}

TopoDS_Shape OccApp::makeBoolCut(TopoDS_Shape& ts)
{
	myShape = BRepAlgoAPI_Cut(myShape, ts);
	return myShape;
}

TopoDS_Shape OccApp::makeBoolUnion(TopoDS_Shape& ts)
{
	myShape = BRepAlgoAPI_Fuse(myShape, ts);
	return TopoDS_Shape();
}

TopoDS_Shape OccApp::makeBoolCoomon(TopoDS_Shape& ts)
{
	myShape = BRepAlgoAPI_Common(myShape, ts);
	return myShape;
}

TopoDS_Shape OccApp::makeCompound(TopoDS_Shape& ts)
{
	myShape = Compound(myShape, ts);
	return myShape;
}

TopoDS_Shape OccApp::boolCut(TopoDS_Shape ts)
{
	return BRepAlgoAPI_Cut(myShape, ts);
}

TopoDS_Shape OccApp::boolUnion(TopoDS_Shape ts)
{
	return BRepAlgoAPI_Fuse(myShape, ts);
}

TopoDS_Shape OccApp::boolCommon(TopoDS_Shape ts)
{
	return BRepAlgoAPI_Common(myShape, ts);
}

TopoDS_Shape OccApp::boolCut(TopoDS_Shape& ts1, TopoDS_Shape& ts2)
{
	return BRepAlgoAPI_Cut(ts1, ts2);
}

TopoDS_Shape OccApp::boolUnion(TopoDS_Shape& ts1, TopoDS_Shape& ts2)
{
	return BRepAlgoAPI_Fuse(ts1, ts2);
}

TopoDS_Shape OccApp::boolCommon(TopoDS_Shape& ts1, TopoDS_Shape& ts2)
{
	return BRepAlgoAPI_Common(ts1, ts2);
}

TopoDS_Shape OccApp::Compound(TopoDS_Shape& ts1, TopoDS_Shape& ts2)
{
	TopoDS_Compound tc;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(tc);
	aBuilder.Add(tc, ts1);
	aBuilder.Add(tc, ts2);
	return tc;
}

inline void OccApp::boolCut()
{
	myShape = BRepAlgoAPI_Cut(myShape, shapeTmp);
}

inline void OccApp::boolUnion()
{
	myShape = BRepAlgoAPI_Fuse(myShape, shapeTmp);
}

/*
TopoDS_Shape OccApp::stretching(gp_Vec dir1, Standard_Real len1, gp_Vec dir2, Standard_Real len2)
{
	if (face) {
		dir1.Normalize();
		dir1 *= len1;
		tmp = BRepPrimAPI_MakePrism(*face, dir1);
		if (len2 > 0) {
			dir2.Normalize();
			dir2 *= len2;
			TopoDS_Shape ts1 = BRepPrimAPI_MakePrism(*face, dir2);
			tmp = BRepAlgoAPI_Fuse(tmp, ts1);
		}
		aBuilder.Add(CompoundShape, tmp);
	}
	return tmp;
}

TopoDS_Shape OccApp::stretchingTo(gp_Pnt p1)
{
	return TopoDS_Shape();
}

TopoDS_Face* OccApp::makeFace()
{
	face = sketch.getFace();
	return (TopoDS_Face*)face;
}

TopoDS_Wire OccApp::makeWire()
{
	TopoDS_Wire tw = sketch.getWire();
	return tw;
}

TopoDS_Shape OccApp::stretchingCut(gp_Vec dir1, Standard_Real len1, gp_Vec dir2, Standard_Real len2)
{
	stretch(dir1, len1);
	myBody = BRepAlgoAPI_Cut(CompoundShape, tmp);
	if (len2 > 0)
	{
		stretch(dir2, len2);
		myBody = BRepAlgoAPI_Cut(myBody, tmp);
	}
	aBuilder.MakeCompound(CompoundShape);
	aBuilder.Add(CompoundShape, myBody);
	return myBody;
}


TopoDS_Shape OccApp::revolve(gp_Ax1 centerAxis, Standard_Real angle1, Standard_Real angle2)
{
	TopoDS_Shape ts = BRepPrimAPI_MakeRevol(*face, centerAxis, angle1 , false);	
	//aBuilder.Add(CompoundShape, ts);
	if (angle2 > 0) {
		TopoDS_Shape ts1 = BRepPrimAPI_MakeRevol(*face, centerAxis, -angle2, false);
		ts = BRepAlgoAPI_Fuse(ts, ts1);
		//aBuilder.Add(CompoundShape, ts);
	}
	myBody = BRepAlgoAPI_Fuse(CompoundShape, ts);
	//aBuilder.Add(CompoundShape, ts);
	aBuilder.MakeCompound(CompoundShape);
	aBuilder.Add(CompoundShape, myBody);
	sketch.clear();
	return ts;
}

TopoDS_Shape OccApp::makeRevolve(gp_Ax1 centerAxis, Standard_Real angle1, Standard_Real angle2)
{
	TopoDS_Shape ts = BRepPrimAPI_MakeRevol(*face, centerAxis, angle1, false);
	if (angle2 > 0) {
		TopoDS_Shape ts1 = BRepPrimAPI_MakeRevol(*face, centerAxis, -angle2, false);
		ts = BRepAlgoAPI_Fuse(ts, ts1);
	}

	sketch.clear();
	return ts;
}

TopoDS_Shape OccApp::revolveCut(gp_Ax1 axis, Standard_Real angle1, Standard_Real angle2)
{
	revol(axis, angle1);
	myBody = BRepAlgoAPI_Cut(CompoundShape, tmp);

	if (angle2 * angle1 < 0) {
		revol(axis, angle1);
		myBody = BRepAlgoAPI_Cut(myBody, tmp);
	}
	aBuilder.MakeCompound(CompoundShape);
	aBuilder.Add(CompoundShape, myBody);
	sketch.clear();
	return myBody;
}

TopoDS_Shape OccApp::mirrow(gp_Ax2 centerAxis)
{
	TopoDS_Shape ts = mirrow(CompoundShape, centerAxis);
	myBody = BRepAlgoAPI_Fuse(CompoundShape, ts);
	aBuilder.MakeCompound(CompoundShape);
	aBuilder.Add(CompoundShape, myBody);
	return ts;
}

TopoDS_Shape OccApp::mirrow(TopoDS_Shape ts, gp_Ax2 centerAxis)
{
	gp_Trsf aTrsf;
	aTrsf.SetMirror(centerAxis);
	BRepBuilderAPI_Transform aBRepTrsf(ts, aTrsf);
	TopoDS_Shape aMirroredShape = aBRepTrsf.Shape();
	return aMirroredShape;
}

TopoDS_Edge OccApp::drawLine(gp_Pnt p1, gp_Pnt p2)
{

	Handle(Geom_TrimmedCurve) line = GC_MakeSegment(p1, p2);
	TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(line);
	aBuilder.Add(CompoundShape, edge);
	return edge;
	//return TopoDS_Edge();
}

TopoDS_Edge OccApp::drawCircle(gp_Pnt pcenter, Standard_Real r, gp_Vec dir)
{
	gp_Circ cir;
	cir.SetLocation(pcenter);
	cir.SetRadius(r);
	cir.SetAxis(gp_Ax1(pcenter, dir));
	TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(cir);
	aBuilder.Add(CompoundShape, edge);
	return edge;
}

void OccApp::SaveIges(const std::string& igesFile)
{
	IGESControl_Controller::Init();
	IGESControl_Writer igesWriter;
	igesWriter.AddShape(CompoundShape);
	igesWriter.ComputeModel();
	igesWriter.Write(igesFile.c_str());
}


inline TopoDS_Shape OccApp::stretch(gp_Vec dir, Standard_Real len)
{
	if (face && len>0) {
		dir.Normalize();
		dir *= len;
		tmp = BRepPrimAPI_MakePrism(*face, dir);
	}
	return tmp;
}

inline TopoDS_Shape OccApp::revol(gp_Ax1 centerAxis, Standard_Real angle1)
{
	if (!face && angle1 != 0) {
		tmp = BRepPrimAPI_MakeRevol(*face, centerAxis, angle1, false);
	}
	return tmp;
}

*/

inline void Sketch::createWire()
{
	flush();
	geomTmp.Clear();
}

Sketch::Sketch():coordinate(gp_XYZ(0,0,0), gp_Vec(0,0,1))
{
	setCoordinate(coordinate);
}

Sketch::Sketch(gp_Ax2 coor):coordinate(coor)
{
	setCoordinate(coordinate);
}

Sketch::Sketch(gp_Pnt posi, gp_Vec direction):coordinate(posi, direction)
{
	setCoordinate(coordinate);
}

Sketch::Sketch(gp_Pnt posi, gp_Vec dx, gp_Vec dy, gp_Vec dz)
{
	coordinate.SetLocation(posi);
	coordinate.SetXDirection(dx);
	coordinate.SetYDirection(dy);
	coordinate.SetDirection(dz);
	setCoordinate(coordinate);
}

Sketch::~Sketch()
{

}

inline void Sketch::setMtl(const gp_Ax2& coor)
{
	mtl.SetCols(coor.XDirection().XYZ(),coor.YDirection().XYZ(),coor.Direction().XYZ());
}

inline void Sketch::setCoordinate(const gp_Ax2& coordinate)
{
	location = coordinate.Location();
	normal = coordinate.Direction();
	this->coordinate = coordinate;
	setMtl(coordinate);
}

inline void Sketch::setNormal(gp_Vec& norm)
{
	coordinate.SetDirection(norm);
	normal = norm;
	setMtl(coordinate);
}

void Sketch::setCoor(const gp_Ax1& ax, const gp_Ax1& ay, const gp_Ax1& az)
{
	coordinate.SetDirection(az.Direction());
	coordinate.SetXDirection(ax.Direction());
	coordinate.SetYDirection(ay.Direction());
	normal = az.Direction();
	setMtl(coordinate);
}

inline gp_Pnt Sketch::getLocation()
{
	return location;
}

inline gp_Vec Sketch::getNormal()
{
	return normal;
}

TopoDS_Face* Sketch::getFace()
{
	getWire();
	if (wire.IsNull())
		return NULL;
	myFace = BRepBuilderAPI_MakeFace(wire);
	return &myFace;
}

Handle(Geom_Curve) Sketch::drawLine(const gp_Pnt2d& p1, const gp_Pnt2d& p2)
{
	Handle(Geom_TrimmedCurve) line = GC_MakeSegment(getGlobalCoor(p1), getGlobalCoor(p2));	
	edgTmp.Append(BRepBuilderAPI_MakeEdge(line));
	geomTmp.Append(line);

	return line;
}

Handle(Geom_Curve) Sketch::drawCircle(const gp_Pnt2d& pCenter, const Standard_Real& radius)
{
	gp_Pnt pC = getGlobalCoor(pCenter);
	gp_Ax2 axis(pC, coordinate.Direction());
	Handle(Geom_Curve) circle = GC_MakeCircle(gp_Circ(axis, radius)).Value();
	edgTmp.Append(BRepBuilderAPI_MakeEdge(circle));
	geomTmp.Append(circle);
	return circle;
}

Handle(Geom_Curve) Sketch::drawArc(const gp_Pnt2d& pfirst, const gp_Pnt2d& pOn, const gp_Pnt2d& pEnd)
{
	Handle(Geom_TrimmedCurve) arc = GC_MakeArcOfCircle(getGlobalCoor(pfirst), getGlobalCoor(pOn), getGlobalCoor(pEnd));
	edgTmp.Append(BRepBuilderAPI_MakeEdge(arc));
	geomTmp.Append(arc);
	return arc;
}

void Sketch::drawRectangle(const gp_Pnt2d& center, const Standard_Real& length, Standard_Real width)
{
	gp_Pnt2d p[4];
	if (width == 0)
		width = length;

	p[0].SetXY(gp_XY(center.X() - length / 2, center.Y() + width / 2));
	p[1].SetXY(gp_XY(center.X() + length / 2, center.Y() + width / 2));
	p[2].SetXY(gp_XY(center.X() + length / 2, center.Y() - width / 2));
	p[3].SetXY(gp_XY(center.X() - length / 2, center.Y() - width / 2));

	for (Standard_Integer i = 0; i < 4; i++) {
		drawLine(p[i], p[(i + 1) % 4]);
	}
	
}

void Sketch::drawRegularPolygon(const gp_Pnt2d& center, gp_Pnt2d pStart, const Standard_Integer& edgNum)
{
	if (edgNum < 3)
		return;
	gp_Pnt2d pnext;
	Standard_Real gap = 2 * M_PI / edgNum;
	for (Standard_Real i = 0; i < edgNum; i++) {
		pnext = pStart.Rotated(center, gap);
		drawLine(pStart, pnext);
		pStart = pnext;
	}
}

void Sketch::drawRectangle(const gp_Pnt2d& p1, const gp_Pnt2d& p2)
{
	gp_Pnt2d center(gp_XY((p1.XY() + p2.XY()) / 2));
	Standard_Real len = Abs(p1.X() - p2.X());
	Standard_Real wid = Abs(p1.Y() - p2.Y());
	if (len * wid > 0)
		drawRectangle(center, len, wid);
}

void Sketch::mirror(const gp_Ax1& center, const Standard_Boolean& isDelete)
{
	/*
	Handle(Geom_Curve) p, p1;
	Handle(Geom_Geometry) pg;

	NCollection_List<Handle(Geom_Curve)>::iterator it = edgeTmp.end();
	if(!isDelete){
		while (it != edgeTmp.begin()) {
			p = *it;
			pg = p->Mirrored(center);
			pg.DownCast(p1);
			edgeTmp.Append(p1);
			//it--;
		}
		p = *it;
		pg = p->Mirrored(center);
		pg.DownCast(p1);
		edgeTmp.Append(p1);
	}
	else {
		while (it != edgeTmp.begin()) {
			p = *it;
			p->Mirror(center);
			//it = it - 1;
		}
		(*it)->Mirror(center);
	}
	*/
}

void Sketch::rotate(const gp_Pnt2d& center, const Standard_Real& angle, const Standard_Boolean& isDelete)
{

}

TopoDS_Wire* Sketch::getWire()
{
	flush();
	return &wire;
}

void Sketch::clear()
{
	edgTmp.Clear();
	geomTmp.Clear();
}

void Sketch::flush()
{	
	if (edgTmp.Size() == 0)
		return;
	BRepBuilderAPI_MakeWire mkWire;
	for (Standard_Integer i = 0; i < edgTmp.Length(); i++) {
		mkWire.Add(edgTmp[i]);
	}
	wire = mkWire.Wire();
	edgTmp.Clear();
	geomTmp.Clear();
}

