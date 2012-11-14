#if 1

#include <QApplication>
#include "barbarellauitest.h"


int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	BarbarellauiTest tatue;
	tatue.show();
	return app.exec();
}


#else

#include <vtkVersion.h>
#include <vtkSmartPointer.h>

#include <vtkQuadricDecimation.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkProgrammableSource.h>
#include <vtkContourFilter.h>
#include <vtkReverseSense.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkQuadricClustering.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "PCLtoVTK.h"

int main(int argc, char *argv[])
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile ("c:/temp/draufgschissn.pcd", *cloud);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//blocks until the cloud is actually rendered
	viewer.showCloud(cloud);

	while (!viewer.wasStopped ()){
	}
	return 0;
}




#if 0

int main(int argc, char *argv[])
{

	vtkSmartPointer<vtkPolyData> input;

#if 0
	vtkSmartPointer<vtkSphereSource> sphereSource =	vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetThetaResolution(100);
	sphereSource->SetPhiResolution(100);
	sphereSource->Update();
	input = sphereSource->GetOutput();
#else 
	input = vtkSmartPointer<vtkPolyData>::New();

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile("c:/temp/segment-2.pcd", *cloud);

	// point cloud instance for the result
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cleaned (new pcl::PointCloud<pcl::PointXYZRGBA>());
	
	// create the radius outlier removal filter
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> radius_outlier_removal;

	radius_outlier_removal.setInputCloud(cloud);
	radius_outlier_removal.setRadiusSearch (0.05);
	radius_outlier_removal.setMinNeighborsInRadius (80);
	radius_outlier_removal.filter (*cleaned);

	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	sor.setInputCloud(cleaned);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);

	PCLtoGenericVTKPolyData(*cleaned, input);
#endif

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(input->GetPoints());

	// Construct the surface and create isosurface.	
	vtkSmartPointer<vtkSurfaceReconstructionFilter> surf = 
		vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();
	surf->SetInput(polydata);

	vtkSmartPointer<vtkContourFilter> cf = 
		vtkSmartPointer<vtkContourFilter>::New();
	cf->SetInputConnection(surf->GetOutputPort());
	//cf->SetValue(0, 0.0);

//  	vtkSmartPointer<vtkQuadricDecimation> decimate =
//  		vtkSmartPointer<vtkQuadricDecimation>::New();
//  	decimate->SetInputConnection(cf->GetOutputPort());
//  	decimate->Update();

	// Sometimes the contouring algorithm can create a volume whose gradient
	// vector and ordering of polygon (using the right hand rule) are
	// inconsistent. vtkReverseSense cures this problem.
	vtkSmartPointer<vtkReverseSense> reverse = 
		vtkSmartPointer<vtkReverseSense>::New();
	reverse->SetInputConnection(cf->GetOutputPort());
	reverse->ReverseCellsOn();
	reverse->ReverseNormalsOn();

	vtkSmartPointer<vtkPolyDataMapper> map = 
		vtkSmartPointer<vtkPolyDataMapper>::New();
	map->SetInputConnection(reverse->GetOutputPort());
	map->ScalarVisibilityOff();

	vtkSmartPointer<vtkActor> surfaceActor = 
		vtkSmartPointer<vtkActor>::New();
	surfaceActor->SetMapper(map);
	surfaceActor->GetProperty()->SetColor(1.0, 0.0, 0.0); //(R,G,B)

	// Create the RenderWindow, Renderer and both Actors
	vtkSmartPointer<vtkRenderer> ren = 
		vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renWin = 
		vtkSmartPointer<vtkRenderWindow>::New();
	renWin->AddRenderer(ren);
	vtkSmartPointer<vtkRenderWindowInteractor> iren = 
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	ren->AddActor(surfaceActor);
	ren->SetBackground(.2, .3, .4);

	renWin->Render();
	iren->Start();

	return EXIT_SUCCESS;
}
#endif

#endif