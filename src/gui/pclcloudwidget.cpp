#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkMatrix4x4.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkTransform.h>
#include <vtkVertexGlyphFilter.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/interactor_style.h>

#include "pclcloudwidget.h"

PclCloudWidget::PclCloudWidget(QWidget *parent)
	: QVTKWidget(parent)
{
	_viewpointTrafo = vtkSmartPointer<vtkMatrix4x4>::New();
	_viewpointTrafo->Identity();

	// vtk internal window
	vtkRenderWindow* renderWindow = this->GetRenderWindow();

	// A renderer and render window
	_renderer = vtkSmartPointer<vtkRenderer>::New();
	renderWindow->AddRenderer(_renderer);

	// An interactor
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
		vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	renderWindowInteractor->SetInteractorStyle(style);
	renderWindowInteractor->Initialize();
	_interactionStyle = ParaView;

	initPipes();
	resetCameraViewpoint();
}

PclCloudWidget::~PclCloudWidget()
{
}

void PclCloudWidget::initPipes()
{
	_cloudActor = vtkSmartPointer<vtkActor>::New ();
	_cloudVtkData = vtkSmartPointer<vtkPolyData>::New();

	// Add 0D topology to every point
	vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexGlyphFilter->SetInputConnection(_cloudVtkData->GetProducerPort());

	// mapper
	vtkSmartPointer<vtkPolyDataMapper> cloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
	cloudMapper->SetInputConnection(vertexGlyphFilter->GetOutputPort());

	cloudMapper->SetScalarRange(0, 256);
	cloudMapper->SetScalarModeToUsePointData();
	cloudMapper->InterpolateScalarsBeforeMappingOn();
	cloudMapper->ScalarVisibilityOn();

	cloudMapper->ImmediateModeRenderingOff();

	_cloudActor->GetProperty()->SetInterpolationToFlat();

	/// FIXME disabling backface culling due to known VTK bug: vtkTextActors are not
	/// shown when there is a vtkActor with backface culling on present in the scene
	/// Please see VTK bug tracker for more details: http://www.vtk.org/Bug/view.php?id=12588
	// actor->GetProperty ()->BackfaceCullingOn ();
	_cloudActor->SetMapper(cloudMapper);

	_cloudActor->GetProperty()->SetInterpolationToFlat();

	_renderer->AddActor(_cloudActor);
}

void PclCloudWidget::updateCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
	convertToVtkMatrix(cloud->sensor_origin_, cloud->sensor_orientation_, _viewpointTrafo);
	pclToGenericVtkPolyData(*cloud, _cloudVtkData);
	GetRenderWindow()->Render();
}

void PclCloudWidget::setCameraParameters (const pcl::visualization::Camera &camera)
{
	vtkSmartPointer<vtkCamera> cam = _renderer->GetActiveCamera();
	cam->SetPosition (camera.pos[0], camera.pos[1], camera.pos[2]);
	cam->SetFocalPoint (camera.focal[0], camera.focal[1], camera.focal[2]);
	cam->SetViewUp (camera.view[0], camera.view[1], camera.view[2]);
	cam->SetClippingRange(camera.clip);
	cam->SetUseHorizontalViewAngle (0);
	cam->SetViewAngle (camera.fovy * 180.0 / M_PI);
	GetRenderWindow()->Render();
}

/** @brief das geht nicht gscheit */
void PclCloudWidget::resetCameraViewpoint()
{
	pcl::visualization::Camera camera_temp;
	// Set default camera parameters to something meaningful
	camera_temp.clip[0] = 0.01;
	camera_temp.clip[1] = 1000.01;

	// Look straight along the z-axis
	camera_temp.focal[0] = 0.;
	camera_temp.focal[1] = 0.;
	camera_temp.focal[2] = 1.;

	// Position the camera at the origin
	camera_temp.pos[0] = 0.;
	camera_temp.pos[1] = 0.;
	camera_temp.pos[2] = 0.;

	// Set the up-vector of the camera to be the y-axis
	camera_temp.view[0] = 0.;
	camera_temp.view[1] = -1.;
	camera_temp.view[2] = 0.;

	// Set the camera field of view to about
	camera_temp.fovy = 0.8575;

 	int *scr_size = GetRenderWindow()->GetScreenSize();
	camera_temp.window_size[0] = scr_size[0]/2;
	camera_temp.window_size[1] = scr_size[1]/2;

	setCameraParameters (camera_temp);
}

void PclCloudWidget::showCloud(bool flag)
{
	_cloudActor->SetVisibility(flag);
}

void PclCloudWidget::setBackgroundColor(const QColor& c)
{
	_renderer->SetBackground(c.redF(), c.greenF(), c.blueF());
}

QColor PclCloudWidget::backgroundColor() const
{
	double *val = _renderer->GetBackground();
	return QColor::fromRgbF(val[0], val[1], val[2]);
}

PclCloudWidget::InteractionStyle PclCloudWidget::interactionSyle() const
{
	return _interactionStyle;
}

void PclCloudWidget::setInteractionStyle(PclCloudWidget::InteractionStyle style)
{
	_interactionStyle = style;

	if(style == ParaView) {
		vtkSmartPointer<vtkInteractorStyleTrackballCamera> s = 
			vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
		GetRenderWindow()->GetInteractor()->SetInteractorStyle(s);
	} else {
		// jess we can
		vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle> s = 
			vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle>::New();
		GetRenderWindow()->GetInteractor()->SetInteractorStyle(s);
	}
}

void PclCloudWidget::convertToVtkMatrix (const Eigen::Vector4f &origin,
										const Eigen::Quaternion<float> &orientation,
										vtkSmartPointer<vtkMatrix4x4> &vtk_matrix)
{
	// set rotation
	Eigen::Matrix3f rot = orientation.toRotationMatrix ();
	for (int i = 0; i < 3; i++)
		for (int k = 0; k < 3; k++)
			vtk_matrix->SetElement (i, k, rot (i, k));

	// set translation
	vtk_matrix->SetElement (0, 3, origin (0));
	vtk_matrix->SetElement (1, 3, origin (1));
	vtk_matrix->SetElement (2, 3, origin (2));
	vtk_matrix->SetElement (3, 3, 1.0f);
}

void PclCloudWidget::setCloudPointSize(int size)
{
	_cloudActor->GetProperty()->SetPointSize(size);
	GetRenderWindow()->Render();
}

float PclCloudWidget::cloudPointSize() const
{
	return _cloudActor->GetProperty()->GetPointSize();
}

void PclCloudWidget::pclToGenericVtkPolyData(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, vtkPolyData* const pdata)
{
	// Coordiantes (always must have coordinates)
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	for (size_t i = 0; i < cloud.points.size (); ++i) {
		double p[3];
		p[0] = cloud.points[i].x;
		p[1] = cloud.points[i].y;
		p[2] = cloud.points[i].z;
		points->InsertNextPoint (p);
	}
	pdata->SetPoints(points);

	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetNumberOfTuples(cloud.points.size());
	colors->SetName("RGB");

	for (size_t i = 0; i < cloud.points.size (); ++i) {
		unsigned char color[3];
		color[0] = cloud.points[i].r;
		color[1] = cloud.points[i].g;
		color[2] = cloud.points[i].b;
		colors->SetTupleValue(i, color);
	}
	pdata->GetPointData()->SetScalars(colors);
}
