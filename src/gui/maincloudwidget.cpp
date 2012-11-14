#include <vtkEventQtSlotConnect.h>
#include <vtkBoxWidget.h>
#include <vtkEventQtSlotConnect.h>

#include "maincloudwidget.h"

MainCloudWidget::MainCloudWidget(QWidget *parent)
	: PclCloudWidget(parent)
{
	_drawMode = NormalMode;

	/****************************** box widget ********************************/
	// The place factor  controls the initial size of the widget with respect to 
	// the bounding box of the input to the widget.
	_boxWidget = vtkSmartPointer<vtkBoxWidget>::New();
	_boxWidget->SetInteractor(this->GetRenderWindow()->GetInteractor());

	// get event data to qt signals/slots world
	_qtconnection = vtkSmartPointer<vtkEventQtSlotConnect>::New();
	_qtconnection->Connect( _boxWidget, vtkCommand::InteractionEvent, 
						    this, SLOT(vtkBoxWidgetInteraction_slot(vtkObject*)) );

	_boxWidget->SetRotationEnabled(false);
	_boxWidget->Off();

	/***************************** Tracking punkt  ********************************/
	vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New ();
	sphereSource->SetRadius (0.1);
	
	_shpereTranform = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
	_shpereTranform->SetInputConnection (sphereSource->GetOutputPort());

	vtkSmartPointer<vtkTransform> trf = vtkSmartPointer<vtkTransform>::New();
	trf->Identity(); _shpereTranform->SetTransform(trf);

	vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
	sphereMapper->SetInputConnection(_shpereTranform->GetOutputPort());

	_sphereActor = vtkSmartPointer<vtkActor>::New ();
	_sphereActor->SetMapper(sphereMapper);
	_sphereActor->SetVisibility(false);
	_sphereActor->GetProperty()->SetColor(0,0,1); // blau und so

	_renderer->AddActor(_sphereActor );

	/****************************** Boden  **********************************/
	_planeSource = vtkSmartPointer<vtkPlaneSource>::New ();
	_planeSource ->SetCenter(1.0, 0.0, 0.0);
	_planeSource ->SetNormal(1.0, 0.0, 1.0);
	_planeSource ->Update();

	vtkSmartPointer<vtkPolyDataMapper> planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
	planeMapper->SetInputConnection(_planeSource->GetOutputPort());

	_planeActor = vtkSmartPointer<vtkActor>::New ();
	_planeActor->SetMapper(planeMapper);
	_planeActor->GetProperty()->SetColor(0,0,1); // blau und so
}

MainCloudWidget::~MainCloudWidget()
{

}

void MainCloudWidget::setSelectionCube(SelectionCube& cube)
{
	vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();

	Eigen::Vector3f pos = cube.getPosition();
	t->Translate(pos[0], pos[1], pos[2]);

	t->Scale(cube.getSx(), cube.getSy(), cube.getSz());
	_boxWidget->SetTransform(t);
	GetRenderWindow()->Render();
}

void MainCloudWidget::getSelectionCube(SelectionCube* cube) const
{
	vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
	_boxWidget->GetTransform(t);

	float pos[3];
	t->GetPosition(pos);
	Eigen::Vector3f position(pos[0], pos[1], pos[2]);
	cube->setPosition(position);

	float s[3];
	t->GetScale(s);
	cube->setSx(s[0]);
	cube->setSx(s[1]);
	cube->setSx(s[2]);
}

void MainCloudWidget::setSelectionCube(double x, double y, double z, 
									   double scaleX, double scaleY, double scaleZ)
{
	vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
	t->Translate(x, y, z);
	t->Scale(scaleX, scaleY, scaleZ);
	_boxWidget->SetTransform(t);
	GetRenderWindow()->Render();
}

void MainCloudWidget::getSelectionCube(double *x, double *y, double *z, 
									   double *scaleX, double *scaleY, double *scaleZ)
{
	vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
	_boxWidget->GetTransform(t);
	double pos[3];
	t->GetPosition(pos);
	(*x) = pos[0];
	(*y) = pos[1];
	(*z) = pos[2];

	double s[3];
	t->GetScale(s);
	(*scaleX) = s[0];
	(*scaleY) = s[1];
	(*scaleZ) = s[2];
}

void MainCloudWidget::vtkBoxWidgetInteraction_slot(vtkObject* caller)
{
	vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
	vtkBoxWidget *widget = reinterpret_cast<vtkBoxWidget*>(caller);
	widget->GetTransform(t);
	double pos[3];
	t->GetPosition(pos);
	double s[3];
	t->GetScale(s);

	emit selectionCubeChanged(pos[0], pos[1], pos[2], s[0], s[1], s[2]);
}

MainCloudWidget::DrawMode MainCloudWidget::drawMode() const
{
	return _drawMode;
}

void MainCloudWidget::setDrawMode(MainCloudWidget::DrawMode mode)
{
	_drawMode = mode;
	
	if(mode == TemplateMode) {
		_boxWidget->On();
		_sphereActor->SetVisibility(false);
		_planeActor->SetVisibility(false);
	} else if (mode == TrackingMode) {
		_boxWidget->Off();
		_sphereActor->SetVisibility(true);
		_planeActor->SetVisibility(false);
	} else if(mode == FloorMode) {
		_boxWidget->Off();
		_sphereActor->SetVisibility(false);
		_planeActor->SetVisibility(true);
	} else {
		_boxWidget->Off();
		_sphereActor->SetVisibility(false);
		_planeActor->SetVisibility(false);
	}
	GetRenderWindow()->Render();
}

void MainCloudWidget::setTrackedCenter(const Eigen::Vector3f& v)
{	
	vtkSmartPointer<vtkTransform> trf = vtkSmartPointer<vtkTransform>::New();
	trf->Translate(v[0], v[1], v[2]);
	_shpereTranform->SetTransform(trf);
}

void MainCloudWidget::setFloor(const pcl::ModelCoefficients &coeff)
{
	_planeSource->SetNormal (coeff.values[0], coeff.values[1], coeff.values[2]);
	double norm_sqr = coeff.values[0] * coeff.values[0]
					  + coeff.values[1] * coeff.values[1]
				      + coeff.values[2] * coeff.values[2];
	_planeSource->Push (- coeff.values[3] / sqrt(norm_sqr));
}
