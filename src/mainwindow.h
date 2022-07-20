#include <QtWidgets/QMainWindow>
#include <QVTKWidget.h>
#include <QPushButton>
#include <QGroupBox>
#include <QGridLayout>
#include <QSlider>
#include <QLineEdit>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <stdlib.h>
#include <cmath>
#include <limits.h>
#include <boost/format.hpp>
 
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
 
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>
 
#include "lccp_segmentation.h"
#include <ctime>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

class mainwindow : public QMainWindow
{
    Q_OBJECT
public:
    mainwindow();
    ~mainwindow();
    void StyleWindow();

protected:
    QVTKWidget* qvtk;
    QPushButton* open;
    QHBoxLayout* hor;
    QVBoxLayout* ver;
    QWidget* rightW;
    QWidget* totalW;

    QLineEdit* lineVoxel;
    QLineEdit* lineSeed;
    QLineEdit* lineColor;
    QLabel* labelVoxel;
    QLabel* labelSeed;
    QLabel* labelColor;

    QGridLayout* verLayout;
    QGroupBox* Box;


private:
	//超体聚类 参数依次是粒子距离、晶核距离、颜色容差、
	float voxel_resolution ;
	float seed_resolution ;
	float color_importance ;
    vtkSmartPointer<vtkRenderWindow> renderWindow;

public slots:
    void lccpSeg();
};