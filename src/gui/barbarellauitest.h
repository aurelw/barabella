#ifndef BARBARELLAUITEST_H
#define BARBARELLAUITEST_H

#ifndef Q_MOC_RUN
#include "ui_barbarellauitest.h"
#include <QtGui/QMainWindow>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#endif


class BarbarellauiTest : public QMainWindow
{
	Q_OBJECT
	Q_ENUMS(GlobalUiMode)
	Q_PROPERTY(GlobalUiMode uiMode READ getUiMode WRITE setUiMode)

public:

	typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
	/** 
	  * @brief globaler modus, der sich mit dem widget auf der linken seite deckt 
	  * @todo namensgebung sub optimal 
	  */
	enum GlobalUiMode {
		Clip = 1,
		SensorAlign,
		DefineRegionOfInterest,
		Track,
		UnknownUIMode
	};

	BarbarellauiTest(QWidget *parent = 0, Qt::WFlags flags = 0);
	~BarbarellauiTest();

	GlobalUiMode getUiMode() const;

public slots:
 	void setUiMode(GlobalUiMode mode);

signals:
	// jemand hat setuimode aufgerufen oder user aht geklickt
	void uiModeChanged(GlobalUiMode newMode);

private slots:

	// category geandert in linker toolbox 
	void on_toolBox_currentChanged(int index);

	// buttons in clip
	void on_btnOpenClip_clicked();
	void on_btnSaveClip_clicked();
	void on_btnClipRecord_clicked();
	void on_btnClipPause_clicked();
	void on_btnClipStop_clicked();

	// boden suchen 
	void on_btnSearchFloor_clicked();
	void on_btnClearSensorAlignment_clicked();

	// region of intereest */
	void on_vtkWidget_selectionCubeChanged(double x, double y, double z, double scaleX, 
										   double scaleY, double scaleZ);

	void on_btnBackgroundColor_clicked();
	void on_btnResetCameraViewport_clicked();

	/** wird getriggert, wenn sich eines eine der spin boxes in Region of interest aendert */
	void boxTransformChanged(double);

	void updatePreviewWidget();

	void on_btnStartTrack_clicked();

private:
	Ui::BarbarellauiTestClass ui;

	QList<QDoubleSpinBox*> transformSpinboxes;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mainCloud;

	QTimer* previewTimer;
};

#endif // BARBARELLAUITEST_H
