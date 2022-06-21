#include "occView.h"

#include <QMenu>
#include <QMouseEvent>
#include <QRubberBand>
#include <QStyleFactory>
#include <QDebug>

// occ header files.
#include <V3d_View.hxx>
//#include <Graphic3d.hxx>
#include <Aspect_Handle.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>

#ifdef WNT
#include <WNT_Window.hxx>
#elif defined(__APPLE__) && !defined(MACOSX_USE_GLX)
#include <Cocoa_Window.hxx>
#else
#include <Xw_Window.hxx>
#endif

#ifndef WNT
#define UNREFERENCED_PARAMETER(p) ((void)(p))
#endif

// the key for multi selection :
#define MULTISELECTIONKEY Qt::ShiftModifier

// the key for shortcut ( use to activate dynamic rotation, panning )
#define CASCADESHORTCUTKEY Qt::ControlModifier

static Handle(Graphic3d_GraphicDriver)& GetGraphicDriver()
{
    static Handle(Graphic3d_GraphicDriver) aGraphicDriver;
    return aGraphicDriver;
}

OccView::OccView(QWidget* parent)
    : QGLWidget(parent),
    mXmin(0),
    mYmin(0),
    mXmax(0),
    mYmax(0),
    mCurrentMode(CurAction3d_DynamicPanning),
    mDegenerateModeIsOn(Standard_True),
    mRectBand(NULL)
{
    setMouseTracking(true);
    init();
}

void OccView::init()
{
    // Create Aspect_DisplayConnection
    Handle(Aspect_DisplayConnection) aDisplayConnection =
        new Aspect_DisplayConnection();

    // Get graphic driver if it exists, otherwise initialise it
    if (GetGraphicDriver().IsNull())
    {

        GetGraphicDriver() = new OpenGl_GraphicDriver(aDisplayConnection);//Graphic3d::InitGraphicDriver(aDisplayConnection);
    }

    // Get window handle. This returns something suitable for all platforms.
    WId window_handle = winId();

    // Create appropriate window for platform
#ifdef WNT
    Handle_WNT_Window wind = new WNT_Window((Aspect_Handle)window_handle);
#elif defined(__APPLE__) && !defined(MACOSX_USE_GLX)
    Handle_Cocoa_Window wind = new Cocoa_Window((NSView *)window_handle);
#else
    Handle_Xw_Window wind = new Xw_Window(aDisplayConnection, (Window)window_handle);
#endif

    // Create V3dViewer and V3d_View
    mViewer = new V3d_Viewer(GetGraphicDriver(), Standard_ExtString("viewer3d"));

    mView = mViewer->CreateView();

    try
    {
        mView->SetWindow(wind);
    }
    catch (...)
    {

    }

    if (!wind->IsMapped()) wind->Map();

    // Create AISInteractiveContext
    mContext = new AIS_InteractiveContext(mViewer);

    // Set up lights etc
    mViewer->SetDefaultLights();
    mViewer->SetLightOn();

    Quantity_Color topColor;
    //topColor.SetValues(double(91.0 / 255), double(91.0 / 255), double(132.0 / 255), Quantity_TOC_RGB);
    topColor.SetValues(double(1), double(1), double(1), Quantity_TOC_RGB);
    Quantity_Color botColor;
    botColor.SetValues(double(161.0 / 255.0), double(161.0 / 255), double(184.0 / 255), Quantity_TOC_RGB);
    //botColor.SetValues(double(1), double(1), double(1), Quantity_TOC_RGB);
    mView->SetBgGradientColors(topColor, botColor,
        Aspect_GFM_VER);

    //mView->SetBackgroundColor(Quantity_NOC_BLACK);
    mView->MustBeResized();
    mView->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_BLACK, 0.08, V3d_ZBUFFER);
//     mView->EnableGLLight();
    mContext->SetDisplayMode(AIS_Shaded, Standard_True);

    // Enable the mouse tracking, by default the mouse tracking is disabled.
}

// Handle_AIS_InteractiveContext OccView::getContext() const
// {
//     return mContext;
// }

void OccView::UpdateView()
{
    mView->MustBeResized();
    mView->Update();
}

Handle_V3d_View OccView::View3d()
{
    return mView;
}

void OccView::paintEvent(QPaintEvent* e)
{
    UNREFERENCED_PARAMETER(e);
    if (mContext.IsNull())
    {
        init();
    }
    mView->Redraw();
    //     mView->Update();
}

void OccView::resizeEvent(QResizeEvent* e)
{

    UNREFERENCED_PARAMETER(e);
    if (!mView.IsNull())
    {
        mView->MustBeResized();
        QWidget::resizeEvent(e);
    }
    //QWidget::resizeEvent(e);
}

void OccView::fitAll(void)
{
    mView->FitAll();
    mView->ZFitAll();
    mView->Redraw();
}

void OccView::reset(void)
{
    mView->Reset();
}

void OccView::pan(void)
{
    mCurrentMode = CurAction3d_DynamicPanning;
}

void OccView::zoom(void)
{
    mCurrentMode = CurAction3d_DynamicZooming;
}

void OccView::rotate(void)
{
    mCurrentMode = CurAction3d_DynamicRotation;
}

void OccView::OnBUTTONAxo()
{
    int i = 19;//V3d_XposYnegZpos
    V3d_TypeOfOrientation iView = (V3d_TypeOfOrientation)i;
    mView->SetImmediateUpdate(false);
    mView->SetProj(iView);
    mView->SetImmediateUpdate(true);
    mView->FitAll();
}

void OccView::OnBUTTONTop()
{
    mView->SetImmediateUpdate(false);
    mView->SetProj(V3d_Zpos);
    mView->SetImmediateUpdate(true);
    mView->FitAll();
}

void OccView::OnBUTTONBack()
{
    mView->SetImmediateUpdate(false);
    mView->SetProj(V3d_Xneg);
    mView->SetImmediateUpdate(true);
    mView->FitAll();
}

void OccView::OnBUTTONBottom()
{
    mView->SetImmediateUpdate(false);
    mView->SetProj(V3d_Zneg);
    mView->SetImmediateUpdate(true);
    mView->FitAll();
}

void OccView::OnBUTTONFront()
{
    mView->SetImmediateUpdate(false);
    mView->SetProj(V3d_Xpos);
    mView->SetImmediateUpdate(true);
    mView->FitAll();
}

void OccView::OnBUTTONLeft()
{
    mView->SetImmediateUpdate(false);
    mView->SetProj(V3d_Yneg);
    mView->SetImmediateUpdate(true);
    mView->FitAll();
}

void OccView::OnBUTTONRight()
{
    mView->SetImmediateUpdate(false);
    mView->SetProj(V3d_Ypos);
    mView->SetImmediateUpdate(true);
    mView->FitAll();
}

void OccView::mousePressEvent(QMouseEvent* e)
{
    if (e->button() == Qt::LeftButton)
    {
        onLButtonDown((e->buttons() | e->modifiers()), e->pos());
        QWidget::mousePressEvent(e);
    }
    else if (e->button() == Qt::MidButton)
    {
        onMButtonDown((e->buttons() | e->modifiers()), e->pos());
    }
    else if (e->button() == Qt::RightButton)
    {
        onRButtonDown((e->buttons() | e->modifiers()), e->pos());
    }
}

void OccView::mouseReleaseEvent(QMouseEvent* e)
{
    if (e->button() == Qt::LeftButton)
    {
        onLButtonUp(e->buttons() | e->modifiers(), e->pos());
    }
    else if (e->button() == Qt::MidButton)
    {
        onMButtonUp(e->buttons() | e->modifiers(), e->pos());
    }
    else if (e->button() == Qt::RightButton)
    {
        onRButtonUp(e->buttons() | e->modifiers(), e->pos());
    }
}

void OccView::mouseMoveEvent(QMouseEvent * e)
{
    onMouseMove(e->buttons(), e->pos());
}

void OccView::wheelEvent(QWheelEvent * e)
{
    onMouseWheel(e->buttons(), e->delta(), e->pos());
}

void OccView::mouseDoubleClickEvent(QMouseEvent *event)
{
    emit mouseDoubleClick(event);
}

void OccView::onLButtonDown(const int theFlags, const QPoint thePoint)
{
    UNREFERENCED_PARAMETER(theFlags);

    // Save the current mouse coordinate in min.
    mXmin = thePoint.x();
    mYmin = thePoint.y();
    mXmax = thePoint.x();
    mYmax = thePoint.y();

    mView->StartRotation(thePoint.x(), thePoint.y());
}

void OccView::onMButtonDown(const int theFlags, const QPoint thePoint)
{
    UNREFERENCED_PARAMETER(theFlags);

    // Save the current mouse coordinate in min.
    mXmin = thePoint.x();
    mYmin = thePoint.y();
    mXmax = thePoint.x();
    mYmax = thePoint.y();

    if (mCurrentMode == CurAction3d_DynamicRotation)
    {
        mView->StartRotation(thePoint.x(), thePoint.y());
    }
}

void OccView::onRButtonDown(const int theFlags, const QPoint thePoint)
{
    UNREFERENCED_PARAMETER(theFlags);
    UNREFERENCED_PARAMETER(thePoint);
}

void OccView::onMouseWheel(const int theFlags, const int theDelta, const QPoint thePoint)
{
    UNREFERENCED_PARAMETER(theFlags);

    Standard_Integer aFactor = 16;

    Standard_Integer aX = thePoint.x();
    Standard_Integer aY = thePoint.y();

    if (theDelta > 0)
    {
        aX += aFactor;
        aY += aFactor;
    }
    else
    {
        aX -= aFactor;
        aY -= aFactor;
    }

    mView->Zoom(thePoint.x(), thePoint.y(), aX, aY);
}

void OccView::addItemInPopup(QMenu* theMenu)
{
    UNREFERENCED_PARAMETER(theMenu);
}

void OccView::popup(const int x, const int y)
{
    UNREFERENCED_PARAMETER(x);
    UNREFERENCED_PARAMETER(y);
}

void OccView::onLButtonUp(const int theFlags, const QPoint thePoint)
{
    // Hide the QRubberBand
    if (mRectBand)
    {
        mRectBand->hide();
    }

    // Ctrl for multi selection.
    if (thePoint.x() == mXmin && thePoint.y() == mYmin)
    {
        if (theFlags & Qt::ControlModifier)
        {
            multiInputEvent(thePoint.x(), thePoint.y());
        }
        else
        {
            inputEvent(thePoint.x(), thePoint.y());
        }
    }

}

void OccView::onMButtonUp(const int theFlags, const QPoint thePoint)
{
    UNREFERENCED_PARAMETER(theFlags);

    if (thePoint.x() == mXmin && thePoint.y() == mYmin)
    {
        panByMiddleButton(thePoint);
    }
}

void OccView::onRButtonUp(const int theFlags, const QPoint thePoint)
{
    UNREFERENCED_PARAMETER(theFlags);

    popup(thePoint.x(), thePoint.y());
}

void OccView::onMouseMove(const int theFlags, const QPoint thePoint)
{
    // Draw the rubber band.
    if (theFlags & Qt::LeftButton)
    {
        //drawRubberBand(mXmin, mYmin, thePoint.x(), thePoint.y());

        //dragEvent(thePoint.x(), thePoint.y());

        mView->Rotation(thePoint.x(), thePoint.y());
    }

    // Ctrl for multi selection.
    if (theFlags & Qt::ControlModifier)
    {
        multiMoveEvent(thePoint.x(), thePoint.y());
    }
    else
    {
        moveEvent(thePoint.x(), thePoint.y());
    }

    // Middle button.
    if (theFlags & Qt::MidButton)
    {
        switch (mCurrentMode)
        {
        case CurAction3d_DynamicRotation:
            mView->Rotation(thePoint.x(), thePoint.y());
            break;

        case CurAction3d_DynamicZooming:
            mView->Zoom(mXmin, mYmin, thePoint.x(), thePoint.y());
            break;

        case CurAction3d_DynamicPanning:
            mView->Pan(thePoint.x() - mXmax, mYmax - thePoint.y());
            mXmax = thePoint.x();
            mYmax = thePoint.y();
            break;

        default:
            break;
        }
    }

}

void OccView::dragEvent(const int x, const int y)
{
    mContext->Select(mXmin, mYmin, x, y, mView, Standard_True);

    emit selectionChanged();
}

void OccView::multiDragEvent(const int x, const int y)
{
    mContext->ShiftSelect(mXmin, mYmin, x, y, mView, Standard_True);

    emit selectionChanged();

}

void OccView::inputEvent(const int x, const int y)
{
    UNREFERENCED_PARAMETER(x);
    UNREFERENCED_PARAMETER(y);

    mContext->Select(Standard_True);

    emit selectionChanged();
}

void OccView::multiInputEvent(const int x, const int y)
{
    UNREFERENCED_PARAMETER(x);
    UNREFERENCED_PARAMETER(y);

    mContext->ShiftSelect(Standard_True);

    emit selectionChanged();
}

void OccView::moveEvent(const int x, const int y)
{
    mContext->MoveTo(x, y, mView, Standard_True);
}

void OccView::multiMoveEvent(const int x, const int y)
{
    mContext->MoveTo(x, y, mView, Standard_True);
}

void OccView::drawRubberBand(const int minX, const int minY, const int maxX, const int maxY)
{
    QRect aRect;

    // Set the rectangle correctly.
    (minX < maxX) ? (aRect.setX(minX)) : (aRect.setX(maxX));
    (minY < maxY) ? (aRect.setY(minY)) : (aRect.setY(maxY));

    aRect.setWidth(abs(maxX - minX));
    aRect.setHeight(abs(maxY - minY));

    if (!mRectBand)
    {
        mRectBand = new QRubberBand(QRubberBand::Rectangle, this);

        // setStyle is important, set to windows style will just draw
        // rectangle frame, otherwise will draw a solid rectangle.
        mRectBand->setStyle(QStyleFactory::create("windows"));
    }

    mRectBand->setGeometry(aRect);
    mRectBand->show();
}

void OccView::panByMiddleButton(const QPoint& thePoint)
{
    Standard_Integer aCenterX = 0;
    Standard_Integer aCenterY = 0;

    QSize aSize = size();

    aCenterX = aSize.width() / 2;
    aCenterY = aSize.height() / 2;

    mView->Pan(aCenterX - thePoint.x(), thePoint.y() - aCenterY);
}
