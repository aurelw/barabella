#ifndef MAINCLOUDWIDGET_H
#define MAINCLOUDWIDGET_H

#ifndef Q_MOC_RUN
#include "pclcloudwidget.h"
#include "selection_cube.h"
#endif

/* forward declerations */
class vtkEventQtSlotConnect;
class vtkBoxWidget;
class vtkSphereSource;
class vtkTransformPolyDataFilter;
class vtkPlaneSource;

class MainCloudWidget : public PclCloudWidget
{
	Q_OBJECT
	Q_ENUMS(DrawMode);

public:

	/** @note same as View3D::DarwaMode */
	enum DrawMode {
		NormalMode,
		FloorMode,		/// @todo namesngebung 
		TemplateMode,
		TrackingMode
	};

	MainCloudWidget(QWidget *parent);
	~MainCloudWidget();

	void setSelectionCube(/*const*/ SelectionCube& cube);
	void getSelectionCube(SelectionCube* cube) const;

	void setSelectionCube(double x, double y, double z, double scaleX, double scaleY, double scaleZ);
	void getSelectionCube(double *x, double *y, double *z, double *scaleX, double *scaleY, double *scaleZ);

	void setFloor(const pcl::ModelCoefficients &coeff);
	void setTrackedCenter(const Eigen::Vector3f& v);

	DrawMode drawMode() const;

public slots:
	void setDrawMode(MainCloudWidget::DrawMode mode);

signals:
	void selectionCubeChanged(double x, double y, double z, double scaleX, double scaleY, double scaleZ);

private slots:
	/* private slot wenn user an der bounding box zieht */
	void vtkBoxWidgetInteraction_slot(vtkObject* caller);

private:
	/* widgets */
	vtkSmartPointer<vtkBoxWidget> _boxWidget;
	vtkSmartPointer<vtkEventQtSlotConnect> _qtconnection;

	/* sphere fuer mittelpunkt trackken */
	vtkSmartPointer<vtkActor> _sphereActor;
	vtkSmartPointer<vtkTransformPolyDataFilter> _shpereTranform;

	/* floor */
	vtkSmartPointer<vtkPlaneSource> _planeSource;
	vtkSmartPointer<vtkActor> _planeActor;
	
	DrawMode _drawMode;
};

#endif // MAINCLOUDWIDGET_H
