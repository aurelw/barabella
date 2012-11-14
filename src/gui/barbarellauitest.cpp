#include <QTimer>
#include <QColorDialog>

#include "barbarellauitest.h"

#include <pcl/io/pcd_io.h>

BarbarellauiTest::BarbarellauiTest(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	
	mainCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);

	previewTimer = new QTimer(this);
	previewTimer->setSingleShot(true);
	connect(previewTimer, SIGNAL(timeout()), this, SLOT(updatePreviewWidget()));

	transformSpinboxes.append(ui.xPosition);
	transformSpinboxes.append(ui.yPosition);
	transformSpinboxes.append(ui.zPosition);
	transformSpinboxes.append(ui.width);
	transformSpinboxes.append(ui.height);
	transformSpinboxes.append(ui.deep);

	foreach(QDoubleSpinBox * cur, transformSpinboxes) {
		connect(cur, SIGNAL(valueChanged (double)), this, SLOT(boxTransformChanged(double)));
	}

	connect(ui.sldrCloudPointSize, SIGNAL(valueChanged (int)), ui.vtkWidget, SLOT(setCloudPointSize(int)) );

}

BarbarellauiTest::~BarbarellauiTest()
{
}

BarbarellauiTest::GlobalUiMode BarbarellauiTest::getUiMode() const
{
	switch(ui.toolBox->currentIndex())
	{
		case 0: return Clip;
		case 1: return SensorAlign;
		case 2: return DefineRegionOfInterest;
		case 3: return Track;
		default: return UnknownUIMode;
	}
}

void BarbarellauiTest::setUiMode(GlobalUiMode mode)
{
	ui.toolBox->setCurrentIndex((int)mode);
}

void BarbarellauiTest::on_toolBox_currentChanged(int)
{
	GlobalUiMode m = getUiMode();

	switch(m)
	{
	case DefineRegionOfInterest:
		ui.vtkWidget->setDrawMode(MainCloudWidget::TemplateMode);
		break;
	case Track:
		ui.vtkWidget->setDrawMode(MainCloudWidget::TrackingMode);
		break;
	case SensorAlign:
		ui.vtkWidget->setDrawMode(MainCloudWidget::FloorMode);
		break;
	default:
		ui.vtkWidget->setDrawMode(MainCloudWidget::NormalMode);
		break;
	}
	emit uiModeChanged(m);
}

/*
clip knoepfe
*/
void BarbarellauiTest::on_btnOpenClip_clicked()
{
	/* datei laden */
	pcl::io::loadPCDFile("c:/temp/draufgschissn.pcd", *mainCloud);
	ui.vtkWidget->updateCloud(mainCloud);
}

void BarbarellauiTest::on_btnSaveClip_clicked()
{
	ui.vtkWidget->resetCameraViewpoint();

}

void BarbarellauiTest::on_btnClipRecord_clicked()
{

}

void BarbarellauiTest::on_btnClipPause_clicked()
{

}

void BarbarellauiTest::on_btnClipStop_clicked()
{

}

/*
Kamera ausrichtung 
*/
void BarbarellauiTest::on_btnSearchFloor_clicked()
{

}

void BarbarellauiTest::on_btnClearSensorAlignment_clicked()
{

}

void BarbarellauiTest::on_vtkWidget_selectionCubeChanged(double x, double y, double z, 
	double scaleX, double scaleY, double scaleZ)
{
	// signale blocken dass diese nicht selbst valueChanhged triggern
	foreach(QDoubleSpinBox * cur, transformSpinboxes) {
		cur->blockSignals(true);
	}

	ui.xPosition->setValue(x);
	ui.yPosition->setValue(y);
	ui.zPosition->setValue(z);
	ui.width->setValue(scaleX);
	ui.height->setValue(scaleY);
	ui.deep->setValue(scaleZ);

	foreach(QDoubleSpinBox * cur, transformSpinboxes) {
		cur->blockSignals(false);
	}

	previewTimer->start(1000);
}

void BarbarellauiTest::boxTransformChanged(double)
{
	double x = ui.xPosition->value();
	double y = ui.yPosition->value();
	double z = ui.zPosition->value();

	double sx = ui.width->value();
	double sy = ui.height->value();
	double sz = ui.deep->value();

	ui.vtkWidget->setSelectionCube(x, y, z, sx, sy, sz);

	previewTimer->start(1000);
}

void BarbarellauiTest::updatePreviewWidget()
{
// weiss nicht aurel, das scheint nicht zu funktionieren
//  	SelectionCube cube;
//  	ui.vtkWidget->getSelectionCube(&cube);
//  	Cloud::Ptr preview = cube.filterCloud(*mainCloud);
//  	ui.roiCloud->updateCloud(preview);
}

void BarbarellauiTest::on_btnBackgroundColor_clicked()
{
	QColorDialog dlg;
	if(dlg.exec() == QDialog::Accepted) {
		QColor c = dlg.currentColor();
		ui.vtkWidget->setBackgroundColor(c);
		ui.roiCloud->setBackgroundColor(c);
	}
}

void BarbarellauiTest::on_btnResetCameraViewport_clicked()
{
	ui.vtkWidget->resetCameraViewpoint();
	ui.roiCloud->resetCameraViewpoint();
}

void BarbarellauiTest::on_btnStartTrack_clicked()
{
	// start trackking 
}
