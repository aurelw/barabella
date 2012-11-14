#ifndef MAINVTKWIDGET_H
#define MAINVTKWIDGET_H

#include <QColor>
#include <QVTKWidget.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkSmartPointer.h>

/* cölass forward declerations */
class vtkMatrix4x4;
class vtkActor;
class vtkPolyData;
class vtkRenderer;

/**
 * @brief basisklasse fuer point cloud widgets
 */
class PclCloudWidget : public QVTKWidget
{
	Q_OBJECT
	Q_ENUMS(InteractionStyle)

	Q_PROPERTY(QColor backgroundColor READ backgroundColor WRITE setBackgroundColor)
	Q_PROPERTY(InteractionStyle interactionSyle READ interactionSyle WRITE setInteractionStyle)
	Q_PROPERTY(float cloudPointSize READ cloudPointSize WRITE setCloudPointSize)

public:

	/**
	 * wie soll sich der fenster bezueglich maus und keystrokes verhalten
	 */
	enum InteractionStyle {
		PointCloudLib,		///< mouse bindings wie bei PCLVizualizer
		ParaView			///< mouse bindings wie bei paraview (nur minimal anders)
	};

	PclCloudWidget(QWidget *parent);
	~PclCloudWidget();

	/** geklaut von PCLVisualizer */
	void setCameraParameters(const pcl::visualization::Camera &camera);

	/** get background color of render window */
	QColor backgroundColor() const;

	/** */
	float cloudPointSize() const;

	/** */ 
	InteractionStyle interactionSyle() const;
	void setInteractionStyle(InteractionStyle style);

	/** update cloud 
	  * @note ist thread save
	  */
	void updateCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);

public slots:

	/** show main coud, hides main cloud if onOff is \c false  */
	void showCloud(bool onOff = true);

	/** @brief hintergrundfarbe vom render fenster setzen */
	void setBackgroundColor(const QColor& c);

	/** load camera viewmatrix from kinect data */
	void resetCameraViewpoint(); 

	/** groesse fuer einen Punnkt in der Punktwolke */
	void setCloudPointSize(int);

protected:
	/* one and only _renderer in vtk sceene */
	vtkSmartPointer<vtkRenderer> _renderer;

private:
	void initPipes();

	/* vtk objects */
	vtkSmartPointer<vtkActor> _cloudActor;
	vtkSmartPointer<vtkPolyData> _cloudVtkData;

	/* */ 
	vtkSmartPointer<vtkMatrix4x4> _viewpointTrafo;

	static void convertToVtkMatrix(const Eigen::Vector4f &origin,
								   const Eigen::Quaternion<float> &orientation,
								   vtkSmartPointer<vtkMatrix4x4> &vtk_matrix);

	static void pclToGenericVtkPolyData(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, 
									    vtkPolyData* const pdata);

	InteractionStyle _interactionStyle;
};

#endif // MAINVTKWIDGET_H
